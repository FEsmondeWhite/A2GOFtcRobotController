package org.firstinspires.ftc.teamcode.OpModes;

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

import java.util.ArrayList;
import java.util.List;

/**
 * ppAutoBase: Sequential Autonomous Base Class
 * Implements a strict "Drive -> Intake -> Reverse -> Sort -> Shoot" handshake logic.
 */
abstract public class ppSimpleAutoBase extends OpMode {

    // --- SUBSYSTEMS ---
    private TimingOptimization timingSystem;
    private List<LynxModule> allHubs;
    private Follower follower;
    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    private Launcher launcher;
    private LED redLED;

    // --- LOGIC & TIMER MANAGEMENT ---
    private final ArrayList<PathChain> pathA = new ArrayList<>();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime taskTimer = new ElapsedTime();   // Stages (Intake/Reverse)
    private final ElapsedTime shotTimer = new ElapsedTime();   // Shooting safety

    private int intakeStage = 0;      // 0: Drive, 1: Reverse, 2: Sort
    private int shotsFired = 0;
    private boolean stageStarted = false; // Guard for state initialization
    private boolean isIndexing = false;   // Sorter/Launcher handshake flag

    // --- CONFIGURATION ---
    private boolean isTestMode = true; // Toggle for home testing (disables drive)
    private final int AllianceColor;
    private final int StartPosition;
    private double shootSpeedRPS;

    // --- COORDINATES ---
    public static Pose startingPose;
    private Pose shootPose, endPose;
    private Pose stack1StartPose, stack1Ball1Pose, stack1Ball2Pose, stack1Ball3Pose;

    public ppSimpleAutoBase(int AllianceColor, int StartPosition) {
        this.AllianceColor = AllianceColor;
        this.StartPosition = StartPosition;
        initializeCoordinates();
    }

    // --- OPMODE LIFECYCLE ---

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        timingSystem = new TimingOptimization(telemetry);
        timingSystem.init();

        intake = new Intake(); intake.init(hardwareMap);
        lifter = new Lifter(); lifter.init(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry); sorter.init(lifter);
        launcher = new Launcher(); launcher.init(hardwareMap);
        redLED = hardwareMap.get(LED.class, "lockdown_LED1");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startingPose);

        if (isTestMode) follower.setMaxPower(0);
    }

    @Override
    public void start() {
        matchTimer.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        timingSystem.update();
        follower.update();
        sorter.update();
        lifter.update();
        launcher.update();

        // Allow state machine to advance instantly if desk-testing
        if (isTestMode) follower.breakFollowing();

        updatePath();

        if (timingSystem.do_telemetry()) {
            telemetry.addData("State", currentState);
            telemetry.addData("Intake Stage", intakeStage);
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.update();
        }
        PoseStorage.currentPose = follower.getPose();
    }

    // --- MAIN STATE MACHINE ---

    enum State { IDLE, SHOOT_A, TRAJ_2, TRAJ_3, TRAJ_4, TRAJ_5, TRAJ_6, SHOOT_B, TRAJ_7, STOP }
    State currentState = State.IDLE;

    public void updatePath() {
        // Emergency stop/park logic
        if (matchTimer.seconds() >= 28.0 && currentState != State.STOP && currentState != State.TRAJ_7) {
            forcePark();
            return;
        }

        switch (currentState) {
            case IDLE:
                launcher.setNominalRPS(shootSpeedRPS);
                launcher.enableMotor();
                follower.followPath(pathA.get(1), true);
                currentState = State.SHOOT_A;
                shotsFired = 0;
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

            // Sequential Intake Steps
            case TRAJ_3: performIntakeStep(3, State.TRAJ_4); break;
            case TRAJ_4: performIntakeStep(4, State.TRAJ_5); break;
            case TRAJ_5: performIntakeStep(5, State.TRAJ_6); break;

            case TRAJ_6:
                if (!follower.isBusy() || isTestMode) {
                    launcher.enableMotor();
                    follower.setMaxPower(1.0);
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

    // --- SUBSYSTEM CONTROL HANDSHAKES ---

    /**
     * Handles the specific "Wait for Launcher -> Verify Sorter -> Fire" handshake.
     */
    private void handleSequentialShooting() {
        if (shotsFired < 3) {
            // Step 1: Ensure sorter has a ball in position
            if (!isIndexing && !lifter.isBusy()) {
                sorter.start(1);
                isIndexing = true;
            }

            // Step 2: Fire only if Sorter is finished AND Launcher is at speed
            if (isIndexing && !sorter.isBusy() && launcher.isReady() && !lifter.isBusy()) {
                lifter.start();
                shotsFired++;
                isIndexing = false;
            }
        } else if (!lifter.isBusy()) {
            // Transition out once all 3 balls are cleared
            if (currentState == State.SHOOT_A) currentState = State.TRAJ_2;
            else {
                follower.followPath(pathA.get(7), true);
                currentState = State.TRAJ_7;
            }
        }
    }

    /**
     * Multi-stage Intake logic: Drive -> Pulse Reverse -> Index (Sort)
     */
    private void performIntakeStep(int pathIdx, State nextState) {
        switch (intakeStage) {
            case 0: // Drive to the ball
                if (!stageStarted) {
                    intake.setIntake(5.0);
                    if (!isTestMode) {
                        follower.setMaxPower(0.3);
                        follower.followPath(pathA.get(pathIdx), true);
                    } else {
                        taskTimer.reset();
                    }
                    stageStarted = true;
                }

                boolean arrived = isTestMode ? (taskTimer.seconds() > 2.0) : !follower.isBusy();
                if (arrived) {
                    intake.setIntake(-2.0); // Quick reverse pulse
                    taskTimer.reset();
                    intakeStage = 1;
                }
                break;

            case 1: // Reverse Pulse
                if (taskTimer.milliseconds() > 300) {
                    intake.setIntake(0);
                    sorter.start(1); // Begin indexing ball
                    intakeStage = 2;
                }
                break;

            case 2: // Indexing
                if (!sorter.isBusy()) {
                    intakeStage = 0;
                    stageStarted = false;
                    currentState = nextState;
                }
                break;
        }
    }

    // --- UTILITIES ---

    private void forcePark() {
        shutdownSubsystems();
        follower.setMaxPower(1.0);
        follower.followPath(pathA.get(7), true);
        currentState = State.TRAJ_7;
    }

    private void shutdownSubsystems() {
        launcher.disableMotor();
        intake.setIntake(0);
    }

    private void initializeCoordinates() {
        double fSpeed = 56.0; double rSpeed = 57.0;
        if (AllianceColor == 1) { // Blue
            startingPose = (StartPosition == 1) ? new Pose(57.5, 9, 1.57079) : new Pose(33.5, 134, 1.57079);
            shootPose = (StartPosition == 1) ? new Pose(59, 14, Math.toRadians(115)) : new Pose(58, 85, Math.toRadians(135));
            shootSpeedRPS = (StartPosition == 1) ? fSpeed : rSpeed;
            endPose = (StartPosition == 1) ? new Pose(48, 25, 1.57079) : new Pose(48, 128, 4.71239);
            stack1StartPose = new Pose(42, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball1Pose = new Pose(36, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball2Pose = new Pose(31, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
            stack1Ball3Pose = new Pose(26, (StartPosition == 1 ? 36 : 84), Math.toRadians(180));
        } else { // Red
            startingPose = (StartPosition == 1) ? new Pose(144-57.5, 9, 1.57079) : new Pose(144-33.5, 134, 1.57079);
            shootPose = (StartPosition == 1) ? new Pose(86, 14, Math.toRadians(67)) : new Pose(88, 85, Math.toRadians(49));
            shootSpeedRPS = (StartPosition == 1) ? fSpeed : rSpeed;
            endPose = (StartPosition == 1) ? new Pose(96, 25, 1.57079) : new Pose(96, 128, 4.71239);
            stack1StartPose = new Pose(102, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball1Pose = new Pose(108, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball2Pose = new Pose(113, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
            stack1Ball3Pose = new Pose(118, (StartPosition == 1 ? 35 : 83), Math.toRadians(0));
        }
    }

    public void buildPaths() {
        follower.setMaxPower(1.0);
        pathA.add(null);
        pathA.add(follower.pathBuilder().addPath(new BezierLine(startingPose, shootPose)).setLinearHeadingInterpolation(startingPose.getHeading(), shootPose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(shootPose, stack1StartPose)).setLinearHeadingInterpolation(shootPose.getHeading(), stack1StartPose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1StartPose, stack1Ball1Pose)).setLinearHeadingInterpolation(stack1StartPose.getHeading(), stack1Ball1Pose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball1Pose, stack1Ball2Pose)).setLinearHeadingInterpolation(stack1Ball1Pose.getHeading(), stack1Ball2Pose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball2Pose, stack1Ball3Pose)).setLinearHeadingInterpolation(stack1Ball2Pose.getHeading(), stack1Ball3Pose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(stack1Ball3Pose, shootPose)).setLinearHeadingInterpolation(stack1Ball3Pose.getHeading(), shootPose.getHeading()).build());
        pathA.add(follower.pathBuilder().addPath(new BezierLine(shootPose, endPose)).setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading()).build());
    }
}
