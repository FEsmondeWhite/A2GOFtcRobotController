package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.gobot.EnPointe;
import org.firstinspires.ftc.teamcode.gobot.Intake;
import org.firstinspires.ftc.teamcode.gobot.Launcher;
import org.firstinspires.ftc.teamcode.gobot.Lifter;
import org.firstinspires.ftc.teamcode.gobot.PoseStorage;
import org.firstinspires.ftc.teamcode.gobot.Sorter;
import org.firstinspires.ftc.teamcode.gobot.TimingOptimization;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

abstract public class ppTeleopBase extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private boolean lockdownMode = false;
    private Pose lockdownPose;
    private final double HOLD_TOLERANCE_INCHES = 0.2;
    private final double HOLD_TOLERANCE_DEGREES = 1.0;
    private boolean robotCentric;

    private Supplier<PathChain> pathChainFront;
    private Supplier<PathChain> pathChainRear;
    private Supplier<PathChain> pathChainPark;
    private Supplier<PathChain> pathChainHuman;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;
    private double driveSpeedMax;
    private double fullSpeed = 1.0;
    private double moderateSpeed = 0.7;

    private boolean autoFlywheelSpeed = true;
    private int AllianceColor;
    private int StartPosition;

    public Pose startingPose;
    public Pose frontShootPose;
    public Pose rearShootPose;
    public Pose ParkPose;
    public Pose HumanPose;
    public Pose GoalPose;
    public Pose ResetPose;

    LED redLED;
    private TimingOptimization timingSystem;
    List<LynxModule> allHubs;

    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    private Launcher launcher;
    private EnPointe enpoint;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    Pose currentPose;
    double currentHeading;
    boolean followerBusy;
    private boolean lastRedLedState;
    private boolean desiredRedLedState;

    public ppTeleopBase(int AllianceColor, int StartPosition) {
        this.AllianceColor = AllianceColor;
        this.StartPosition = StartPosition;

        // --- POSE INITIALIZATION ---

        if (AllianceColor == 1) { // Blue
            if (StartPosition == 1) {
                startingPose = new Pose(57.5, 9, 1.57079); // Math.toRadians(90)
            } else if (StartPosition == 2) {
                startingPose = new Pose(29.5, 134, 4.71239);
            }
            frontShootPose = new Pose(71,20,2.11185);
            rearShootPose = new Pose(71,79, 2.40855);
            ParkPose = new Pose(110,38, Math.toRadians(135)); // parking pose for blue. Facing toward goal, with robot on team side.
            HumanPose = new Pose(127.5,15.5, 4.71239);
            GoalPose = new Pose(0, 144);
            ResetPose = new Pose(134, 9, Math.toRadians(90));
        } else { // Red. AllianceColor is 2
            if (StartPosition == 1) {
                startingPose = new Pose(144 - 57.5, 9, 1.57079);
            } else if (StartPosition == 2) {
                startingPose = new Pose(144 - 29.5, 134, 4.71239);
            }
            frontShootPose = new Pose(144-71,20, 1.02974);
            rearShootPose = new Pose(144-71,79, 0.73304);
            ParkPose = new Pose(34.8,29, Math.toRadians(-45)); // parking pose for blue. Facing toward goal, with robot on team side.
            HumanPose = new Pose(16.5,15.5, 1.57079);
            GoalPose = new Pose(144, 144);
            ResetPose = new Pose(9, 9, Math.toRadians(90));
        }

        // If there is a stored pose (starting from PP auto) then get the pose from storage
        if (PoseStorage.currentPose != null) {
            // This will load the pose if we have already run the robot.
            startingPose = PoseStorage.currentPose;
        }
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        timingSystem = new TimingOptimization(telemetry);
        timingSystem.init();

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        // These must be instantiated and have a value.
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        redLED = hardwareMap.get(LED.class, "lockdown_LED1");
        redLED.enable(false);
        lastRedLedState = false;

        intake = new Intake(); intake.init(hardwareMap);
        lifter = new Lifter(); lifter.init(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry); sorter.init(lifter);
        launcher = new Launcher(); launcher.init(hardwareMap);
        enpoint = new EnPointe(); enpoint.init(hardwareMap);
        if (StartPosition == 1) {
            launcher.setNominalRPS(56);
        } else if (StartPosition == 2) {
            launcher.setNominalRPS(56);
        } // Front vs Back

        robotCentric = true;
        automatedDrive = false;
        autoFlywheelSpeed = false;

        follower = Constants.createFollower(hardwareMap);
        driveSpeedMax = fullSpeed;
        follower.setMaxPower(driveSpeedMax);
        follower.setStartingPose(startingPose);
        follower.update();

        // --- PATH SUPPLIERS ---
        this.pathChainRear = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(currentPose, rearShootPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(() -> currentHeading, rearShootPose.getHeading(), 0.8))
                .build();

        this.pathChainFront = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(currentPose, frontShootPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(() -> currentHeading, frontShootPose.getHeading(), 0.8))
                .build();

        this.pathChainPark = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(currentPose, ParkPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(() -> currentHeading, ParkPose.getHeading(), 0.8))
                .build();

        this.pathChainHuman = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(currentPose, HumanPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(() -> currentHeading, HumanPose.getHeading(), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // --- READ SECTION ---
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }

        follower.update();
        currentPose = follower.getPose();
        currentHeading = follower.getHeading();
        followerBusy = follower.isBusy();

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        timingSystem.update();

        // --- THINK SECTION (Logic & State Machines) ---

        // 1. MANUALLY DRIVEN OVERRIDE (Applies to all automation)
        boolean isManualInput = Math.abs(currentGamepad1.left_stick_x) > 0.2 ||
                Math.abs(currentGamepad1.left_stick_y) > 0.2 ||
                Math.abs(currentGamepad1.right_stick_x) > 0.2;

        if (isManualInput && (automatedDrive || lockdownMode)) {
            automatedDrive = false;
            lockdownMode = false;
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        // 2. CONSOLIDATED AUTOMATION LOGIC
        if (!enpoint.isBusy()) {
            if (lockdownMode) {
                if (!enpoint.isBusy()) {
                    double distanceError = currentPose.distanceFrom(lockdownPose);
                    double headingError = Math.abs(currentPose.getHeading() - lockdownPose.getHeading());

                    if ((distanceError > HOLD_TOLERANCE_INCHES || headingError > Math.toRadians(HOLD_TOLERANCE_DEGREES)) && !followerBusy) {
                        // Fix: Create path, set heading, THEN follow
                        Path lockdownPath = new Path(new BezierLine(currentPose, lockdownPose));
                        lockdownPath.setConstantHeadingInterpolation(lockdownPose.getHeading());
                        follower.followPath(lockdownPath, true);
                    } else if (distanceError < (HOLD_TOLERANCE_INCHES * 0.8) && followerBusy) {
                        follower.breakFollowing();
                    }
                }
            } else if (automatedDrive) {
                if (!followerBusy || currentGamepad1.xWasPressed()) {
                    automatedDrive = false;
                    follower.startTeleopDrive();
                }
            } else {
            // 3. MANUAL DRIVE SECTION (Only if not automated)
                if (currentGamepad1.optionsWasReleased()) { // Allow toggling robotCentric
                    robotCentric = !robotCentric;
                }

                double x = robotCentric ? -currentGamepad1.left_stick_y : currentGamepad1.left_stick_x;
                double y = robotCentric ? -currentGamepad1.left_stick_x : -currentGamepad1.left_stick_y;
                double rot = -currentGamepad1.right_stick_x;
                double multiplier = slowMode ? slowModeMultiplier : 1.0;

                follower.setTeleOpDrive(x * multiplier, y * multiplier, rot * multiplier, robotCentric);
            }
        }

        // --- SUBSYSTEMS & BUTTON TRIGGERS ---

        // Path Triggers
        if (currentGamepad1.dpadDownWasPressed()) { follower.followPath(pathChainFront.get(), true); automatedDrive = true; }
        if (currentGamepad1.dpadUpWasPressed()) { follower.followPath(pathChainRear.get(), true); automatedDrive = true; }
        if (currentGamepad1.dpadLeftWasPressed()) { follower.followPath(pathChainPark.get(), true); automatedDrive = true; }
        if (currentGamepad1.dpadRightWasPressed()) { follower.followPath(pathChainHuman.get(), true); automatedDrive = true; }
        // Toggle Lockdown Mode
        if (currentGamepad1.yWasReleased()) {
            if (lockdownMode) {
                // TURN OFF: If already in lockdown, cancel it
                lockdownMode = false;
                automatedDrive = false;
                follower.breakFollowing();
                follower.startTeleopDrive();
            } else {
                // TURN ON: Capture pose and engage
                lockdownPose = currentPose;
                lockdownMode = true;
                automatedDrive = true;
            }
        }
        // Sorter/Intake/Launcher
        if (currentGamepad1.a) intake.setIntake(5); else if (currentGamepad1.b) intake.setIntake(-2); else intake.setIntake(0);

        if (!lifter.isBusy()) {
            if (currentGamepad2.rightBumperWasPressed()) sorter.start(1);
            if (currentGamepad2.left_bumper) sorter.startUnbind();
        }
        if (!sorter.isBusy() && currentGamepad2.right_trigger > 0.5) lifter.start();

        // 2. Pose Reset (Moved to Share)
        if (currentGamepad2.shareWasReleased()) {
            follower.setPose(ResetPose);
        }

        if (currentGamepad2.optionsWasReleased()) {
            autoFlywheelSpeed = !autoFlywheelSpeed;
        }

        // Flywheel
        if (!autoFlywheelSpeed) {
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)
            {
                launcher.setNominalRPS(launcher.getNominalRPS()+0.5);
            }
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)
            {
                launcher.setNominalRPS(launcher.getNominalRPS()-0.5);
            }
            if (currentGamepad2.dpad_right)
            {
                launcher.setNominalRPS(57);
            }
            if (currentGamepad2.dpad_left)
            {
                launcher.setNominalRPS(56);
            }
        } else {
            // 2. Automatic Distance Calculation
            if (autoFlywheelSpeed) {
//                if (!autoFlywheelSpeed) launcher.setNominalRPS(launcher.getSpeedNearestToDistance(launcher.getDistance(currentPose, GoalPose)));
                double dist = launcher.getDistance(currentPose, GoalPose);
                double targetSpeed = launcher.getSpeedNearestToDistance(dist);
                if (targetSpeed != -1) {
                    launcher.setNominalRPS(targetSpeed);
                }
            }
        }

        if (currentGamepad2.xWasPressed()) {
            launcher.enableMotor();
        } else if (currentGamepad2.xWasReleased()) {
            launcher.disableMotor();
        }

        if (currentGamepad2.yWasReleased()) { if (enpoint.isBusy()) enpoint.stop(); else enpoint.start(); }

        // Speed Toggles
        if (currentGamepad1.leftBumperWasPressed()) {
            driveSpeedMax = (driveSpeedMax == fullSpeed) ? moderateSpeed : fullSpeed;
            follower.setMaxPower(driveSpeedMax);
        }
        slowMode = currentGamepad1.right_bumper;

        // --- LED FEEDBACK ---
        if (lockdownMode) {
            desiredRedLedState = (System.currentTimeMillis() % 200) < 100;
        } else {
            desiredRedLedState = automatedDrive;
        }

        if (desiredRedLedState != lastRedLedState) {
            redLED.enable(desiredRedLedState);
            lastRedLedState = desiredRedLedState;
        }

        // --- ACT SECTION ---
        sorter.update();
        lifter.update();
        launcher.update();
        enpoint.update();

        // --- TELEMETRY ---
        if (timingSystem.do_telemetry()) {// Display Setpoint (Active) and Actual (Measured)
            telemetry.addData("Launcher Mode", autoFlywheelSpeed ? "AUTO" : "MANUAL");
            telemetry.addData("Launcher", "Setpoint: %.1f | Actual: %.1f RPS",
                    launcher.getActiveSetpointRPS(),
                    launcher.actual_RPS);

            // If you want to see the stored "Nominal" even when off
            telemetry.addData("Launcher Nominal", "%.1f RPS", launcher.getNominalRPS());

//            this.sorter.balls.detailedTelemetry();
            this.sorter.balls.telemetry();
            telemetry.update();
        }
    }
}
