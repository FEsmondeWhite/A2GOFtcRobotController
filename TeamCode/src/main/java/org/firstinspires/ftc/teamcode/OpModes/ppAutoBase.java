package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Size;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gobot.Intake;
import org.firstinspires.ftc.teamcode.gobot.Launcher;
import org.firstinspires.ftc.teamcode.gobot.Lifter;
import org.firstinspires.ftc.teamcode.gobot.PoseStorage;
import org.firstinspires.ftc.teamcode.gobot.Sorter;
import org.firstinspires.ftc.teamcode.gobot.TimingOptimization;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.gobot.VisionDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * ppAutoBase: Optimized Autonomous Base Class
 * Incorporates:
 * - Reactive pattern-based scoring with Safety/Panic overrides.
 * - Performance-gated telemetry via TimingOptimization.
 * - Manual Bulk Caching for high-frequency control loops.
 */

/**
 * AUTO ROUTINE OVERVIEW:
 * * 1.  INITIALIZATION & SETUP
 * - Configures high-frequency I2C bulk caching (Manual Mode) for all hardware hubs.
 * - Initializes performance monitoring (TimingOptimization) to gate telemetry updates.
 * - Builds a sequence of PedroPathing trajectories based on Alliance Color and Start Position.
 * * 2.  PHASE 1: VISION SCAN & INITIAL SCORE (TRAJ_0 -> SHOOT_A)
 * - Navigates to an AprilTag scanning position to identify the specific artifact pattern.
 * - Injects the detected color sequence into the Sorter system for reactive filtering.
 * - Moves to the shooting position and launches the first set of artifacts.
 * * 3.  PHASE 2: REACTIVE SHOOTING LOGIC
 * - Color-Sorting: Automated sorting cycles spit out incorrect colors until the pattern match is found.
 * - Safety Launch: Prevents robot stalls; if a ball is detected but doesn't match for 1.5s, it fires anyway.
 * - Pattern Tracking: Physically launching an item increments the sequence to the next required color.
 * * 4.  PHASE 3: FIELD COLLECTION (TRAJ_2 -> TRAJ_6)
 * - Travels through a "Stack Collection" path consisting of three distinct pickup points.
 * - Employs a specific "Burst Reverse" on the intake at each stop to ensure proper ball seating.
 * - Lowers drive power during intake steps (0.25 max) to maximize collection reliability.
 * * 5.  PHASE 4: FINAL SCORE & PARK (SHOOT_B -> TRAJ_7)
 * - Returns to the scoring pose after collection to clear the remaining inventory.
 * - Navigates to a designated end-pose to park within the final seconds of the match.
 * * 6.  SAFETY & OPTIMIZATION
 * - Global Abort: At 28 seconds, all subsystems shutdown, and the robot forces a park (TRAJ_7).
 * - Telemetry Gating: Throttles dashboard updates to 4Hz to preserve CPU cycles for pathing.
 * - LED Feedback: Provides visual status of whether the robot is currently processing a path or task.
 */

abstract public class ppAutoBase extends OpMode {

    // --- Subsystems & Performance ---
    private TimingOptimization timingSystem;
    private List<LynxModule> allHubs;
    private Follower follower;
    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    private Launcher launcher;
    private VisionDetection vision;
    private LED redLED;

    // --- Vision and Pattern Logic ---
    private int patternID = 0;
    private List<Character> colorPattern = null;

    // --- State & Pathing ---
    private final ArrayList<PathChain> pathA = new ArrayList<>();
    private ElapsedTime taskTimer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime matchTimer = new ElapsedTime();
    private ElapsedTime safetyTimer = new ElapsedTime();

    public static Pose startingPose;
    private Pose shootPose, scanAprilTagPose, endPose;
    private Pose stack1StartPose, stack1Ball1Pose, stack1Ball2Pose, stack1Ball3Pose;

    private int consecutiveMatchCount = 0;
    private int sortCycleCount = 0;
    private char lastSeenOutColor = 'N';
    private boolean isBallStuck = false;
    private final int AllianceColor;
    private final int StartPosition;
    private double shootSpeedRPS;
    private boolean lastRedLedState = false;

    enum TaskState { TASK_IDLE, TASK_LIFTING, TASK_SORTING, TASK_SCANNING }
    TaskState taskState = TaskState.TASK_IDLE;

    enum State { IDLE, TRAJ_0, SCAN, TRAJ_1, SHOOT_A, TRAJ_2, TRAJ_3, TRAJ_4, TRAJ_5, TRAJ_6, SHOOT_B, TRAJ_7, STOP }
    State currentState = State.IDLE;

    public ppAutoBase(int AllianceColor, int StartPosition) {
        this.AllianceColor = AllianceColor;
        this.StartPosition = StartPosition;
        initializeCoordinates();
    }

    @Override
    public void init() {
        // 1. Setup Bulk Caching for all Hubs (I2C Optimization)
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Performance Monitor
        timingSystem = new TimingOptimization(telemetry);
        timingSystem.init();

        // 3. Subsystem Initialization
        intake = new Intake(); intake.init(hardwareMap);
        lifter = new Lifter(); lifter.init(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry); sorter.init(lifter);
        launcher = new Launcher(); launcher.init(hardwareMap);
        redLED = hardwareMap.get(LED.class, "lockdown_LED1");

        vision = new VisionDetection(hardwareMap, AllianceColor, true, new Size(640, 480), 30, 255);
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startingPose);
    }

    @Override
    public void start() {
        matchTimer.reset();
        follower.startTeleopDrive(); // Required for PedroPathing movement initialization
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }
        timingSystem.update();

        follower.update();
        sorter.update();
        lifter.update();
        launcher.update();

        updatePath();

        if (timingSystem.do_telemetry()) {
            telemetry.addData("Auto State", currentState);
            telemetry.addData("Task State", taskState);

            // Safety check: Only show pattern info if it's been initialized by vision
            if (colorPattern != null) {
                telemetry.addData("Pattern ID", patternID);
                telemetry.addData("Target Index", sorter.balls.getPatternIndex());
            }

            this.sorter.balls.telemetry();
            telemetry.update();
        }

        PoseStorage.currentPose = follower.getPose();
    }

    public void updatePath() {
        // --- EMERGENCY SHUTDOWN / PARK ---
        if (matchTimer.seconds() >= 28.0 && currentState != State.STOP && currentState != State.TRAJ_7) {
            shutdownSubsystems();
            follower.followPath(pathA.get(7), true);
            currentState = State.TRAJ_7;
            return;
        }

        // --- LED Feedback Management ---
        boolean desiredLED = (currentState != State.IDLE && currentState != State.STOP);
        if (desiredLED != lastRedLedState) {
            redLED.enable(desiredLED);
            lastRedLedState = desiredLED;
        }

        switch (currentState) {
            case IDLE: currentState = State.TRAJ_0; break;

            case TRAJ_0:
                if (!follower.isBusy()) {
                    prepareForScoring();
                    follower.followPath(pathA.get(0), true);
                    currentState = State.SCAN;
                }
                break;

            case SCAN:
                if (!follower.isBusy()) handleVisionScan();
                break;

            case TRAJ_1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(pathA.get(1), true);
                    currentState = State.SHOOT_A;
                    stateTimer.reset();
                }
                break;

            case SHOOT_A:
            case SHOOT_B:
                if (!follower.isBusy()) handleShootingLogic();
                break;

            case TRAJ_2:
                if (!follower.isBusy()) {
                    follower.followPath(pathA.get(2), true);
                    currentState = State.TRAJ_3;
                }
                break;

            case TRAJ_3: performIntakeStep(3, State.TRAJ_4); break;
            case TRAJ_4: performIntakeStep(4, State.TRAJ_5); break;
            case TRAJ_5: performIntakeStep(5, State.TRAJ_6); break;

            case TRAJ_6:
                if (!follower.isBusy() && taskTimer.seconds() > 1.0) {
                    intake.setIntake(0);
                    launcher.enableMotor();
                    follower.setMaxPower(1.0);
                    follower.followPath(pathA.get(6), true);
                    currentState = State.SHOOT_B;
                    stateTimer.reset();
                }
                break;

            case TRAJ_7:
                if (!follower.isBusy()) {
                    follower.followPath(pathA.get(7), true);
                    currentState = State.STOP;
                }
                break;

            case STOP: shutdownSubsystems(); break;
        }
    }

    // --- REFACTORED LOGIC HELPERS ---

    private void handleShootingLogic() {
        char currentOut = this.sorter.balls.getColorList().get(2);
        boolean ballAtExit = (currentOut != 'N');

        // Filter sensor noise
        if (currentOut == lastSeenOutColor) consecutiveMatchCount++;
        else { consecutiveMatchCount = 0; lastSeenOutColor = currentOut; }

        switch (taskState) {
            case TASK_IDLE:
                boolean isMatch = this.sorter.balls.targetMatchForLaunch() && consecutiveMatchCount >= 2;

                // Safety/Panic Override: If a ball is at the exit but doesn't match for 1.5s, fire it anyway.
                if (ballAtExit && !isBallStuck) { safetyTimer.reset(); isBallStuck = true; }
                if (!ballAtExit) isBallStuck = false;
                boolean safetyLaunch = isBallStuck && safetyTimer.milliseconds() > 1500;

                if ((isMatch || safetyLaunch) && launcher.isReady() && !sorter.isBusy()) {
                    lifter.start();
                    taskState = TaskState.TASK_LIFTING;
                    stateTimer.reset();
                } else if (ballAtExit && !sorter.isBusy() && !lifter.isBusy()) {
                    if (sortCycleCount >= 6) moveToNextTrajectory();
                    else { sorter.start(1); sortCycleCount++; taskState = TaskState.TASK_SORTING; }
                } else if (!ballAtExit && stateTimer.milliseconds() > 700) {
                    moveToNextTrajectory(); // No balls left
                }
                break;

            case TASK_LIFTING:
                if (!lifter.isBusy() && stateTimer.milliseconds() > 400) {
                    this.sorter.balls.incrementPatternIndex();
                    resetShootingVars();
                }
                break;

            case TASK_SORTING:
                if (!sorter.isBusy()) taskState = TaskState.TASK_IDLE;
                break;
        }
    }

    private void performIntakeStep(int pathIdx, State nextState) {
        if (!follower.isBusy()) {
            if (taskState == TaskState.TASK_IDLE) {
                sorter.start(1);
                intake.setIntake(-2);
                taskState = TaskState.TASK_SORTING;
            } else if (taskState == TaskState.TASK_SORTING && !sorter.isBusy()) {
                intake.setIntake(5);
                follower.setMaxPower(0.25);
                follower.followPath(pathA.get(pathIdx), true);
                taskState = TaskState.TASK_IDLE;
                taskTimer.reset();
                currentState = nextState;
            }
        }
    }

    private void handleVisionScan() {
        if (taskState == TaskState.TASK_IDLE) { taskTimer.reset(); taskState = TaskState.TASK_SCANNING; }
        patternID = vision.getDetectedPatternID();

        if (patternID != 0 || taskTimer.milliseconds() > 1000) {
            colorPattern = (patternID == 0) ? Arrays.asList('P', 'G', 'P') : vision.getArtifactColorPattern();
            this.sorter.balls.setPatternList(colorPattern);
            this.sorter.balls.resetPatternIndex();
            currentState = State.TRAJ_1;
            taskState = TaskState.TASK_IDLE;
        }
    }

    private void prepareForScoring() {
        sorter.start(1);
        launcher.setNominalRPS(shootSpeedRPS);
        launcher.enableMotor();
        taskState = TaskState.TASK_IDLE;
    }

    private void resetShootingVars() {
        sortCycleCount = 0; consecutiveMatchCount = 0;
        isBallStuck = false; taskState = TaskState.TASK_IDLE;
        stateTimer.reset();
    }

    private void shutdownSubsystems() { launcher.disableMotor(); intake.setIntake(0); }

    private void moveToNextTrajectory() {
        resetShootingVars();
        if (currentState == State.SHOOT_A) { intake.setIntake(5); currentState = State.TRAJ_2; }
        else currentState = State.TRAJ_7;
    }

    // --- COORDINATE & PATH BUILDING ---

    private void initializeCoordinates() {
        double fSpeed = 52.5; double rSpeed = 57.0;
        if (AllianceColor == 1) { // Blue
            startingPose = (StartPosition == 1) ? new Pose(57.5, 9, 1.57079) : new Pose(33.5, 134, 1.57079);
            shootPose = (StartPosition == 1) ? new Pose(59, 14, Math.toRadians(115)) : new Pose(58, 85, Math.toRadians(135));
            shootSpeedRPS = (StartPosition == 1) ? fSpeed : rSpeed;
            endPose = (StartPosition == 1) ? new Pose(48, 25, 1.57079) : new Pose(48, 128, 4.71239);
            scanAprilTagPose = (StartPosition == 1) ? new Pose(58.5, 25, Math.toRadians(83)) : new Pose(55, 110, Math.toRadians(67));
            stack1StartPose = new Pose(42, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball1Pose = new Pose(36, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball2Pose = new Pose(31, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball3Pose = new Pose(26, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
        } else { // Red
            startingPose = (StartPosition == 1) ? new Pose(144-57.5, 9, 1.57079) : new Pose(144-33.5, 134, 1.57079);
            shootPose = (StartPosition == 1) ? new Pose(86, 14, Math.toRadians(67)) : new Pose(88, 85, Math.toRadians(49));
            shootSpeedRPS = (StartPosition == 1) ? fSpeed : rSpeed;
            endPose = (StartPosition == 1) ? new Pose(96, 25, 1.57079) : new Pose(96, 128, 4.71239);
            scanAprilTagPose = (StartPosition == 1) ? new Pose(85.5, 25, Math.toRadians(97)) : new Pose(90, 110, Math.toRadians(113));
            stack1StartPose = new Pose(102, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball1Pose = new Pose(108, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball2Pose = new Pose(113, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball3Pose = new Pose(118, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
        }
    }

    public void buildPaths() {
        follower.setMaxPower(1.0);
        pathA.add(follower.pathBuilder().addPath(new BezierLine(startingPose, scanAprilTagPose)).setLinearHeadingInterpolation(startingPose.getHeading(), scanAprilTagPose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(scanAprilTagPose, shootPose)).setLinearHeadingInterpolation(scanAprilTagPose.getHeading(), shootPose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(shootPose, stack1StartPose)).setLinearHeadingInterpolation(shootPose.getHeading(), stack1StartPose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1StartPose, stack1Ball1Pose)).setLinearHeadingInterpolation(stack1StartPose.getHeading(), stack1Ball1Pose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball1Pose, stack1Ball2Pose)).setLinearHeadingInterpolation(stack1Ball1Pose.getHeading(), stack1Ball2Pose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball2Pose, stack1Ball3Pose)).setLinearHeadingInterpolation(stack1Ball2Pose.getHeading(), stack1Ball3Pose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball3Pose, shootPose)).setLinearHeadingInterpolation(stack1Ball3Pose.getHeading(), shootPose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(shootPose, endPose)).setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading()).build());
    }
}
