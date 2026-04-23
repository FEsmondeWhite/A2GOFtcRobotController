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

import java.util.ArrayList;
import java.util.List;

/**
 * ppAutoBase: Optimized Sequential Timed Auto
 * - Continuous sorter(1) engagement during intake and shoot indexing.
 * - Precision intake speed (0.3 power) to prevent ball bouncing.
 * - Strict 28s emergency park trigger.
 */
abstract public class ppAutoBase extends OpMode {
    // --- Subsystems ---
    private TimingOptimization timingSystem;
    private List<LynxModule> allHubs;
    private Follower follower;
    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    private Launcher launcher;
    private LED redLED;

    // --- State & Timer Management ---
    private final ArrayList<PathChain> pathA = new ArrayList<>();
    private ElapsedTime matchTimer = new ElapsedTime();
    private ElapsedTime taskTimer = new ElapsedTime(); // Used for intake stages
    private ElapsedTime shotTimer = new ElapsedTime(); // Used for shooting safety

    private boolean taskTimerActive = false;
    private int intakeStage = 0;

    private boolean stageStarted = false;
    private boolean isIndexing = false;
    private int shotsFired = 0;

    // --- Test Mode Config ---
    private boolean isTestMode = true; // Toggle this for home testing

    public static Pose startingPose;
    private Pose shootPose, endPose;
    private Pose stack1StartPose, stack1Ball1Pose, stack1Ball2Pose, stack1Ball3Pose;

    private final int AllianceColor;
    private final int StartPosition;
    private double shootSpeedRPS;

    public ppAutoBase(int AllianceColor, int StartPosition) {
        this.AllianceColor = AllianceColor;
        this.StartPosition = StartPosition;
        initializeCoordinates();
    }

    @Override
    public void init() {
        isTestMode = true;
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

        if (isTestMode) {
            follower.setMaxPower(0); // Prevents movement but allows logic to run
        }
    }

    @Override
    public void start() {
        matchTimer.reset();
        // Do NOT call follower.startTeleopDrive();
        // Just let the state machine in loop() trigger the first path.
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        timingSystem.update();

        follower.update();
        sorter.update();
        lifter.update();
        launcher.update();

        if (isTestMode) {
            // This tricks the 'if (!follower.isBusy())' checks in your updatePath()
            // It makes the state machine think the robot finished every path instantly.
            follower.breakFollowing();
        }

        updatePath();

        if (timingSystem.do_telemetry()) {
            telemetry.addData("Auto State", currentState);
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.addData("Launcher Ready", launcher.isReady());
            telemetry.update();
        }
        PoseStorage.currentPose = follower.getPose();
    }
    // --- State Machine ---
    enum State { IDLE, SHOOT_A, TRAJ_2, TRAJ_3, TRAJ_4, TRAJ_5, TRAJ_6, SHOOT_B, TRAJ_7, STOP }
    State currentState = State.IDLE;

    public void updatePath() {
        // --- 28-SECOND EMERGENCY PARK ---
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
                shotTimer.reset();
                break;

            case SHOOT_A:
            case SHOOT_B:
                // Ensure sorter is constantly helping index while shooting
                if (!follower.isBusy() || isTestMode) {
                    handleSequentialShooting();
                }
                break;

            case TRAJ_2:
                if (!follower.isBusy() || isTestMode) {
                    intake.setIntake(5.0);
                    intakeStage = 0; // Reset intake logic
                    follower.followPath(pathA.get(2));
                    currentState = State.TRAJ_3;
                }
                break;

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
                    shotTimer.reset();
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
        // 1. Check if we still have balls to fire
        if (shotsFired < 3) {

            // PHASE A: Request a ball if we aren't currently moving one
            if (!isIndexing && !lifter.isBusy()) {
                sorter.start(1);
                isIndexing = true;
            }

            // PHASE B: The Handshake
            // We wait for the sorter to finish placing the ball AND the launcher to be at speed
            if (isIndexing && !sorter.isBusy() && launcher.isReady() && !lifter.isBusy()) {
                lifter.start();
                shotsFired++;
                isIndexing = false; // Reset so we can request the next ball after this lift
            }

        }
        // 2. All shots fired? Wait for the final lift to finish before moving the robot
        else if (!lifter.isBusy()) {
            if (currentState == State.SHOOT_A) {
                currentState = State.TRAJ_2;
            } else {
                follower.followPath(pathA.get(7), true);
                currentState = State.TRAJ_7;
            }
        }
    }
    /**
     * Sequential Intake:
     * 0: Drive to ball with active intake/sort.
     * 1: Brief reverse pulse to clear potential jams.
     * 2: Wait for Sorter state machine to finish indexing.
     */
    private void performIntakeStep(int pathIdx, State nextState) {
        switch (intakeStage) {
            case 0: // STAGE: Drive to the ball
                if (!stageStarted) {
                    intake.setIntake(5.0);
                    if (!isTestMode) {
                        follower.setMaxPower(0.3);
                        follower.followPath(pathA.get(pathIdx), true);
                    } else {
                        taskTimer.reset(); // Only need the timer for the 2.0s delay in test mode
                    }
                    stageStarted = true;
                }

                // Arrived logic: Timer-based for home, Follower-based for field
                boolean arrived = isTestMode ? (taskTimer.seconds() > 2.0) : !follower.isBusy();

                if (arrived) {
                    intake.setIntake(-2.0);
                    taskTimer.reset();      // Reset here for the Stage 1 reverse pulse
                    intakeStage = 1;
                }
                break;

            case 1: // STAGE: Brief Reverse pulse
                if (taskTimer.milliseconds() > 300) {
                    intake.setIntake(0);
                    sorter.start(1);
                    intakeStage = 2;
                }
                break;

            case 2: // STAGE: State-driven sort
                if (!sorter.isBusy()) {
                    intakeStage = 0;
                    stageStarted = false; // Reset guard for the next ball
                    currentState = nextState;
                }
                break;
        }
    }

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
