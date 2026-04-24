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
import org.firstinspires.ftc.teamcode.gobot.VisionDetection; // Added
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

abstract public class ppAutoBase extends OpMode {
    private boolean isTestMode = true; // Set to false for actual runs

    // --- SUBSYSTEMS ---
    private TimingOptimization timingSystem;
    private List<LynxModule> allHubs;
    private Follower follower;
    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    private Launcher launcher;
    private VisionDetection vision; // Added
    private LED redLED;

    // --- LOGIC & TIMER MANAGEMENT ---
    private final ArrayList<PathChain> pathA = new ArrayList<>();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime taskTimer = new ElapsedTime();

    private int intakeStage = 0;
    private int shotsFired = 0;
    private boolean stageStarted = false;
    private boolean needsRefresh = false;

    // --- VISION DATA ---
    private int patternID = 0;
    private List<Character> colorPattern;
    private static final boolean MANUAL_CAMERA = true;
    private static final Size CAMERA_RESOLUTION = new Size(640, 480);
    public static int EXPOSURE_MS = 12;
    public static int GAIN_VAL = 255;

    // --- CONFIGURATION ---
    private final int AllianceColor;
    private final int StartPosition;
    private double shootSpeedRPS;

    // --- COORDINATES ---
    public static Pose startingPose;
    private Pose scanAprilTagPose, shootPose, endPose;
    private Pose stack1StartPose, stack1Ball1Pose, stack1Ball2Pose, stack1Ball3Pose;

    // Task Management for Vision
    enum TaskState { TASK_IDLE, TASK_SCANNING }
    TaskState taskState = TaskState.TASK_IDLE;

    public ppAutoBase(int AllianceColor, int StartPosition) {
        this.AllianceColor = AllianceColor;
        this.StartPosition = StartPosition;
        initializeCoordinates();
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        timingSystem = new TimingOptimization(telemetry);
        timingSystem.init();

        intake = new Intake(); intake.init(hardwareMap);
        lifter = new Lifter(); lifter.init(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry); sorter.init(lifter);
        launcher = new Launcher(); launcher.init(hardwareMap);

        // Initialize Vision
        vision = new VisionDetection(hardwareMap, AllianceColor, MANUAL_CAMERA, CAMERA_RESOLUTION, EXPOSURE_MS, GAIN_VAL);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startingPose);

        if (isTestMode) follower.setMaxPower(0);
    }

    @Override
    public void start() {
        // Switch back to MANUAL at start for high-performance loop timing
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        matchTimer.reset();
        launcher.setNominalRPS(shootSpeedRPS);
        launcher.enableMotor();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        timingSystem.update();
        follower.update();
        sorter.update();
        lifter.update();
        launcher.update();

        updatePath();

        if (timingSystem.do_telemetry()) {
            telemetry.addData("State", currentState);
            telemetry.addData("Shots Fired", shotsFired);
            // New Diagnostic Lines
            telemetry.addData("Target Color", sorter.balls.targetPatternColor());
            telemetry.addData("Current Slot 2 (Out)", sorter.balls.getSlot(2));
            sorter.addTelemetry();
            telemetry.update();
        }
        PoseStorage.currentPose = follower.getPose();
    }

    // --- MAIN STATE MACHINE ---

    enum State { IDLE, TRAJ_0, SCAN, TRAJ_1, SHOOT_A, TRAJ_2, TRAJ_3, TRAJ_4, TRAJ_5, TRAJ_6, SHOOT_B, TRAJ_7, STOP }
    State currentState = State.IDLE;

    public void updatePath() {
        if (matchTimer.seconds() >= 28.5 && currentState != State.STOP && currentState != State.TRAJ_7) {
            forcePark();
            return;
        }

        switch (currentState) {
            case IDLE:
                follower.followPath(pathA.get(0), true); // Move to Scan Pose
                // --- CRITICAL FIX FOR ISSUE 1 ---
                // Request a hardware survey of the internal slots NOW.
                // This fills the N, N, N with the actual balls you pre-loaded.
                sorter.balls.requestSurvey(5);
//                needsRefresh = false; // We just started one, so handleSequential doesn't need to trigger another yet.
//                sorter.balls.forceInitialInventory(); // sorter.balls.inventory ={'P', 'G', 'P'}; // Manually override the sensors for testing
                currentState = State.TRAJ_0;
                break;

            case TRAJ_0:
                if (!follower.isBusy() || isTestMode) currentState = State.SCAN;
                break;

            case SCAN:
                if (taskState == TaskState.TASK_IDLE) {
                    taskTimer.reset();
                    taskState = TaskState.TASK_SCANNING;
                }

                patternID = vision.getDetectedPatternID();
                if (patternID != 0 || taskTimer.milliseconds() > 1000) {
                    // Hand off pattern or default
                    colorPattern = (patternID != 0) ? vision.getArtifactColorPattern() : Arrays.asList('P', 'G', 'P');
                    sorter.balls.setPatternList(colorPattern);
                    sorter.balls.resetPatternIndex();

                    follower.followPath(pathA.get(1), true); // Move to Shoot Pose
                    currentState = State.TRAJ_1;
                    taskState = TaskState.TASK_IDLE;
                }
                break;

            case TRAJ_1:
                if ((!follower.isBusy() || isTestMode) && !sorter.balls.isSurveying()) {
                    vision.closeVision(); // Save CPU
                    currentState = State.SHOOT_A;
                }
                break;

            case SHOOT_A:
            case SHOOT_B:
                if (!follower.isBusy() || isTestMode) handleSequentialShooting();
                break;

            case TRAJ_2:
                if (!follower.isBusy() || isTestMode) {
                    intake.setIntake(5.0);
                    intakeStage = 0;
                    follower.followPath(pathA.get(2));
                    currentState = State.TRAJ_3;
                }
                break;

            case TRAJ_3: performIntakeStep(3, State.TRAJ_4); break;
            case TRAJ_4: performIntakeStep(4, State.TRAJ_5); break;
            case TRAJ_5:
                performIntakeStep(5, State.TRAJ_6);
                launcher.enableMotor();
                break;

            case TRAJ_6:
                if (!follower.isBusy() || isTestMode) {
//                    launcher.enableMotor();
                    follower.followPath(pathA.get(6), true);
                    currentState = State.SHOOT_B;
                    shotsFired = 0;
                }
                break;

            case TRAJ_7:
                if (!follower.isBusy() || isTestMode) {
                    shutdownSubsystems();
                    currentState = State.STOP;
                }
                break;

            case STOP: break;
        }
    }

    private void handleSequentialShooting() {
        // 1. HARDWARE SAFETY LOCK
        // If the lifter is up or the carousel is spinning, we wait.
        if (lifter.isBusy() || sorter.isBusy()) return;

        // 2. THE MANDATORY NUDGE
        // This runs immediately after executeLaunch() sets needsRefresh = true.
        // It physically clears the ball and indexes the next slot.
        if (needsRefresh) {
            sorter.start(1);
            needsRefresh = false; // Control is now with the Sorter; it will trigger its own survey.
            return;
        }

        // 3. SENSOR DELAY
        // Wait for the Sorter's internal survey (triggered in Sorter Case 4) to finish.
        if (sorter.balls.isSurveying()) return;

        // 4. MAIN DECISION LOOP
        if (shotsFired < 3) {
            // Optional Diagnostics
            if (timingSystem.do_telemetry()) {
                telemetry.addData("Pattern Target", sorter.balls.targetPatternColor());
                telemetry.addData("Virtual Inventory", Arrays.toString(sorter.balls.getInventory()));
            }

            // --- PRIORITY LOGIC ---

            // A. If the ball in the chamber matches what we want... FIRE.
            if (sorter.balls.targetMatchForLaunch()) {
                if (launcher.isReady()) executeLaunch();
            }
            // B. If the target ball is in the 'In' or 'Hold' slots... ROTATE.
            else if (sorter.balls.targetBallAvailableElsewhere()) {
                sorter.start(1);
                // Note: sorter.start(1) triggers its own survey, so no needsRefresh needed.
            }
            // C. If the chamber has A ball, but not THE ball... FIRE anyway to clear it.
            else if (sorter.balls.anyBallReadyForLaunch()) {
                if (launcher.isReady()) executeLaunch();
            }
            // D. If we have balls but they aren't in the chamber... ROTATE.
            else if (sorter.balls.anyBallAvailable()) {
                sorter.start(1);
            }
            // E. If the sensors literally see nothing left... EXIT.
            else {
                shotsFired = 3;
            }
        }

        // 5. EXIT & CLEANUP
        // This only triggers once shotsFired >= 3 AND the last "nudge" is complete.
        else {
            launcher.disableMotor();
            if (currentState == State.SHOOT_A) {
                currentState = State.TRAJ_2;
            } else {
                // Final park
                follower.followPath(pathA.get(7), true);
                currentState = State.TRAJ_7;
            }
        }
    }

    private void executeLaunch() {
        lifter.start();
        sorter.balls.launch();            // Sets Out slot to 'N'
        sorter.balls.incrementPatternIndex(); // Move to next color in P,G,P
        shotsFired++;
        needsRefresh = true;              // Triggers the "Nudge" in the next loop
    }

    private void performIntakeStep(int pathIdx, State nextState) {
        switch (intakeStage) {
            case 0: // Drive to the ball
                if (!stageStarted) {
                    intake.setIntake(5.0);
                    if (!isTestMode) {
                        follower.setMaxPower(0.4);
                        follower.followPath(pathA.get(pathIdx), true);
                    } else {
                        taskTimer.reset();
                    }
                    stageStarted = true;
                }

                // Dwell for 2 seconds in test mode, otherwise wait for Pedro
                boolean arrived = isTestMode ? (taskTimer.seconds() > 2.0) : !follower.isBusy();

                if (arrived) {
                    intake.setIntake(-2.5); // Reverse pulse to clear jams
                    taskTimer.reset();
                    intakeStage = 1;
                }
                break;

            case 1: // Intake Dwell & Survey Trigger
                if (taskTimer.milliseconds() > 400) {
                    intake.setIntake(0);
                    sorter.balls.requestSurvey(3); // Scan now that ball should be inside
                    sorter.start(1); // Begin indexing
                    intakeStage = 2;
                }
                break;

            case 2: // Wait for Sorter
                if (!sorter.isBusy()) {
                    intakeStage = 0;
                    stageStarted = false;
                    currentState = nextState;
                }
                break;
        }
    }

    private void forcePark() {
        shutdownSubsystems();
        follower.followPath(pathA.get(7), true);
        currentState = State.TRAJ_7;
    }

    private void shutdownSubsystems() {
        launcher.disableMotor();
        intake.setIntake(0);
    }

    private void initializeCoordinates() {
        // scanAprilTagPose must be set before buildPaths()
        if (AllianceColor == 1) { // Blue
            scanAprilTagPose = (StartPosition == 1) ? new Pose(58.5, 25, Math.toRadians(83)) : new Pose(55, 110, Math.toRadians(67));
            startingPose = (StartPosition == 1) ? new Pose(57.5, 9, 1.57079) : new Pose(33.5, 134, 1.57079);
            shootPose = (StartPosition == 1) ? new Pose(59, 14, Math.toRadians(115)) : new Pose(58, 85, Math.toRadians(135));
            shootSpeedRPS = (StartPosition == 1) ? 56.0 : 57.0;
            endPose = (StartPosition == 1) ? new Pose(48, 25, 1.57079) : new Pose(48, 128, 4.71239);
            stack1StartPose = new Pose(42, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball1Pose = new Pose(36, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball2Pose = new Pose(31, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball3Pose = new Pose(26, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
        } else { // Red
            scanAprilTagPose = (StartPosition == 1) ? new Pose(85.5, 25, Math.toRadians(97)) : new Pose(90, 110, Math.toRadians(113));
            startingPose = (StartPosition == 1) ? new Pose(86.5, 9, 1.57079) : new Pose(110.5, 134, 1.57079);
            shootPose = (StartPosition == 1) ? new Pose(86, 14, Math.toRadians(67)) : new Pose(88, 85, Math.toRadians(49));
            shootSpeedRPS = (StartPosition == 1) ? 56.0 : 57.0;
            endPose = (StartPosition == 1) ? new Pose(96, 25, 1.57079) : new Pose(96, 128, 4.71239);
            stack1StartPose = new Pose(102, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball1Pose = new Pose(108, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball2Pose = new Pose(113, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball3Pose = new Pose(118, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
        }
    }

    public void buildPaths() {
        pathA.clear();
        // Index 0: Start -> Scan
        pathA.add(follower.pathBuilder().addPath(new BezierLine(startingPose, scanAprilTagPose)).setLinearHeadingInterpolation(startingPose.getHeading(), scanAprilTagPose.getHeading()).build());
        // Index 1: Scan -> Shoot
        pathA.add(follower.pathBuilder().addPath(new BezierLine(scanAprilTagPose, shootPose)).setLinearHeadingInterpolation(scanAprilTagPose.getHeading(), shootPose.getHeading()).build());
        // Index 2: Shoot -> Stack Start
        pathA.add(follower.pathBuilder().addPath(new BezierLine(shootPose, stack1StartPose)).setLinearHeadingInterpolation(shootPose.getHeading(), stack1StartPose.getHeading()).build());
        // Index 3, 4, 5: Individual Balls
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1StartPose, stack1Ball1Pose)).setConstantHeadingInterpolation(stack1StartPose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball1Pose, stack1Ball2Pose)).setConstantHeadingInterpolation(stack1Ball1Pose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball2Pose, stack1Ball3Pose)).setConstantHeadingInterpolation(stack1Ball2Pose.getHeading()).build());
        // Index 6: Return to Shoot
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball3Pose, shootPose)).setLinearHeadingInterpolation(stack1Ball3Pose.getHeading(), shootPose.getHeading()).build());
        // Index 7: Final Park
        pathA.add(follower.pathBuilder().addPath(new BezierLine(shootPose, endPose)).setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading()).build());
    }
}