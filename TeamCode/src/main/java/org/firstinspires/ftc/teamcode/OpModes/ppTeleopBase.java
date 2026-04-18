package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.gobot.EnPointe;
import org.firstinspires.ftc.teamcode.gobot.Intake;
import org.firstinspires.ftc.teamcode.gobot.Launcher;
import org.firstinspires.ftc.teamcode.gobot.Lifter;
import org.firstinspires.ftc.teamcode.gobot.PoseStorage;
import org.firstinspires.ftc.teamcode.gobot.Sorter;
import org.firstinspires.ftc.teamcode.gobot.TimingOptimization;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.gobot.StartingPose;

import java.util.List;
import java.util.function.Supplier;

//@TeleOp()
abstract public class ppTeleopBase extends OpMode {
    private Follower follower;
    private boolean automatedDrive;

    private int currentBall;

    private boolean lockdownMode = false;
    private Path lockdownPath;
    private Pose lockdownPose;
    private final double HOLD_TOLERANCE_INCHES = 0.2; // originally 0.5
    private final double HOLD_TOLERANCE_DEGREES = 1.0; // originally 2.0
    private boolean robotCentric; // true means robot centric, false is field centric
    private Supplier<PathChain> pathChainFront;
    private Supplier<PathChain> pathChainRear;
    private Supplier<PathChain> pathChainPark;
    private Supplier<PathChain> pathChainHuman;

//    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25; // Was too fast for accurate parking at 0.5
    private double driveSpeedMax; //
    private double fullSpeed = 1.0;
    private double moderateSpeed = 0.7;

    private boolean autoFlywheelSpeed = true;

    private int AllianceColor;
//    private int AllianceColor = 1; // Blue is 1
//    private int AllianceColor = 2; // Red is 2
    private int StartPosition;
//    private int StartPosition = 1; // Front is 1
//    private int StartPosition = 2; // Back is 2

    public Pose startingPose;  //See ExampleAuto to understand how to use this

    public Pose frontShootPose;
    public Pose rearShootPose;
    public Pose ParkPose;
    public Pose HumanPose;
    public Pose GoalPose;
    public Pose ResetPose;


    // Declare a LED object for the indicator LEDs
    LED redLED;

    private TimingOptimization timingSystem;
    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    public Launcher launcher;
    private EnPointe enpoint;

    List<LynxModule> allHubs;


    // Get pose and heading based on alliance color and start position (front/back)
    public ppTeleopBase(int AllianceColor, int StartPosition) {
        this.AllianceColor = AllianceColor;
        this.StartPosition = StartPosition;
        if (AllianceColor == 1) {
            if (StartPosition == 1) {
                startingPose = new Pose(57.5, 9, 1.57079); // Math.toRadians(90)
            } else if (StartPosition == 2) {
                startingPose = new Pose(29.5, 134, 4.71239);
            }
        } else if (AllianceColor == 2) {
            if (StartPosition == 1) {
                startingPose = new Pose(144 - 57.5, 9, 1.57079);
            } else if (StartPosition == 2) {
                startingPose = new Pose(144 - 29.5, 134, 4.71239);
            }
        }

        // If there is a stored pose (starting from PP auto) then
        // Get the pose from storage
        if(!( PoseStorage.currentPose==null) ) {
            // This will load the pose if we have already run the robot.
            startingPose = PoseStorage.currentPose;
        }

        // Define the field poses for shooting and parking based on alliance color
        if (AllianceColor == 1) {
            frontShootPose = new Pose(71,20,2.11185); // Math.toRadians(121));
            rearShootPose = new Pose(71,79, 2.40855); // Math.toRadians(138));
            ParkPose = new Pose(144-38.6,33.4, Math.toRadians(180)); // Math.toRadians(90));
            HumanPose = new Pose(127.5,15.5, 4.71239); // Math.toRadians(270));
            GoalPose = new Pose(0, 144); // Blue goal
            ResetPose = new Pose(134, 9, Math.toRadians(90));
        } else if (AllianceColor == 2) {
            frontShootPose = new Pose(144-71,20, 1.02974); // Math.toRadians(180-121));
            rearShootPose = new Pose(144-71,79, 0.73304); // Math.toRadians(180-138));
            ParkPose = new Pose(38.6,33.4, Math.toRadians(0)); // Math.toRadians(90));
            HumanPose = new Pose(16.5,15.5, 1.57079); // Math.toRadians(90));
            GoalPose = new Pose(144, 144); // Red goal
            ResetPose = new Pose(9, 9, Math.toRadians(90));
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

        intake = new Intake();
        intake.init(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        sorter.init();
        lifter = new Lifter();
        lifter.init(hardwareMap);
        launcher = new Launcher();
        launcher.init(hardwareMap);
        enpoint = new EnPointe();
        enpoint.init(hardwareMap);

        // Initialize the LED from the hardware map
        // The name "myLED" must match the name in your configuration file
        redLED = hardwareMap.get(LED.class, "lockdown_LED1");
        redLED.enable(false);

//        // Set the LED as an output device
//        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
//        redLED.setMode(DigitalChannel.Mode.OUTPUT);

        currentBall = 0; // Set the pattern position to ball 0

        // true means robot centric, false is field centric
        robotCentric = true;

        //Drivetrain (PP)
//        follower=org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        driveSpeedMax = fullSpeed; // fullSpeed or moderateSpeed
        follower.setMaxPower(driveSpeedMax);

        follower.setStartingPose(startingPose);
        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        this.pathChainRear = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, rearShootPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, rearShootPose.getHeading(), 0.8))
                .build();

        this.pathChainFront = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, frontShootPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, frontShootPose.getHeading(), 0.8))
                .build();

        this.pathChainPark = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, ParkPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, ParkPose.getHeading(), 0.8))
                .build();

        this.pathChainHuman = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, HumanPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, HumanPose.getHeading(), 0.8))
                .build();
    }


    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // IMPORTANT: Clear caches for BOTH hubs at the very start
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        timingSystem.update();
        sorter.update();
        lifter.update();
        launcher.update();
        enpoint.update();
        if (timingSystem.do_telemetry()) {
            telemetry.addData("Flywheel speed (RPS) ", launcher.actual_RPS);
        }

        // Intake
        if (gamepad1.a) {
            intake.setIntake(5);
        } else if (gamepad1.b) {
            intake.setIntake(-2);
        } else {
            intake.setIntake(0);
        }

        // Sorter
        if (!lifter.isBusy()) {
            if (gamepad2.rightBumperWasPressed()) {
                sorter.start(1);
            }
        }

        // Lifter
        if (!sorter.isBusy()) {
            if (gamepad2.right_trigger > 0.5) {
                lifter.start();
            }
        }

//        if (gamepad2.left_trigger > 0.5) {
//            lifter.stop();
//        }

        // Flywheel
        if (gamepad2.dpadUpWasReleased())
        {
            launcher.setNominalRPS(launcher.getNominalRPS()+1);
        }
        if (gamepad2.dpadDownWasReleased())
        {
            launcher.setNominalRPS(launcher.getNominalRPS()-1);
        }
        if (gamepad2.dpadRightWasReleased())
        {
            launcher.setNominalRPS(57);
        }
        if (gamepad2.dpadLeftWasReleased())
        {
            launcher.setNominalRPS(52.5);
        }

        if (gamepad2.xWasReleased()) {
            autoFlywheelSpeed = !autoFlywheelSpeed;
        }

        if (autoFlywheelSpeed == false)
        {
            launcher.setNominalRPS(
                    launcher.getSpeedNearestToDistance(
                            launcher.getDistance(follower.getPose(), GoalPose)
                    )
            );
        }

        if (gamepad2.circleWasReleased()) {
            follower.setPose(ResetPose);
        }

//        if (timingSystem.do_telemetry()) {
//            // static public flywheel PIDF allows it to be changed in panels
//            telemetryM.addData("Flywheel setpoint", launcher.getNominalRPS());
//            telemetryM.addData("Flywheel actual", launcher.actual_RPS);
//            }

        if (gamepad2.x) {
            launcher.enableMotor();
            if (timingSystem.do_telemetry()) {
                telemetry.addData("Flywheel setpoint: ", launcher.getNominalRPS());
                telemetry.addData("Flywheel actual: ", launcher.actual_RPS);
            }
        } else {
            launcher.disableMotor();
        }

        //Automated PathFollowing to the front launch position
        if (gamepad1.dpadDownWasPressed()) {
            follower.followPath(this.pathChainFront.get(), true);
            automatedDrive = true;
        }

        //Automated PathFollowing to the rear launch position
        if (gamepad1.dpadUpWasPressed()) {
            follower.followPath(this.pathChainRear.get(), true);
            automatedDrive = true;
        }

        //Automated PathFollowing to the end game park position
        if (gamepad1.dpadLeftWasPressed()) {
            follower.followPath(this.pathChainPark.get(), true);
            automatedDrive = true;
        }

        //Automated PathFollowing to the human player artifact loading position
        if (gamepad1.dpadRightWasPressed()) {
            follower.followPath(this.pathChainHuman.get(), true);
            automatedDrive = true;
        }


        //Automated PathFollowing to the human player artifact loading position
        if (gamepad2.yWasReleased()) {
            if (enpoint.isBusy()) {
                enpoint.stop();
            } else {
                enpoint.start();
            }
        }

        // Start automated lockdown mode to hold position
        if (gamepad1.yWasReleased()) {
            // ENGAGE HOLD: Capture current pose and lock target
            lockdownPose = follower.getPose();
            automatedDrive = true;
            lockdownMode = true;

//            BezierPoint holdPoint = new BezierPoint(lockdownPose);
//            lockdownPath = new Path(new BezierLine(lockdownPose, lockdownPose));
//
//            // 3. Set the heading to remain constant (the current heading)
//            lockdownPath.setConstantHeadingInterpolation(lockdownPose.getHeading());
//
//            // 4. Instruct Follower to follow this path and HOLD the end
//            follower.followPath(lockdownPath, true);
        }

        // Handle Manual Driving
        if (enpoint.isBusy()) {
            // If enpointe is activated, we don't want to run teleop or auto movement.
        } else if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors


            if (gamepad1.optionsWasReleased()) {
                robotCentric = !(robotCentric);
            }
            if (!slowMode) {
                if (!(robotCentric)) {
                    //This is the normal version to use in the TeleOp
                    follower.setTeleOpDrive(
                            gamepad1.left_stick_x,
                            -gamepad1.left_stick_y,
                            -gamepad1.right_stick_x,
                            robotCentric // Field Centric
                    );
                } else {
                    //This is the normal version to use in the TeleOp
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x,
                            robotCentric // Robot Centric
                    );
                }
            }
                //This is how it looks with slowMode on
            else {
                if (!(robotCentric)) {
                    //This is the slow version to use in the TeleOp
                    follower.setTeleOpDrive(
                            gamepad1.left_stick_x * slowModeMultiplier,
                            -gamepad1.left_stick_y * slowModeMultiplier,
                            -gamepad1.right_stick_x * slowModeMultiplier,
                            robotCentric // Field Centric
                    );
                } else {
                    //This is the slow version to use in the TeleOp
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * slowModeMultiplier,
                            -gamepad1.left_stick_x * slowModeMultiplier,
                            -gamepad1.right_stick_x * slowModeMultiplier,
                            robotCentric // Robot Centric
                    );
                }
            }
        }

        if (lockdownMode && !enpoint.isBusy()) {
            if (!automatedDrive) {
                // RELEASE HOLD: Switch back to manual teleop
                follower.breakFollowing();
                follower.startTeleopDrive();
                lockdownMode = false;
            } else {
                // Calculate how far we are from the locked spot
                double distanceError = follower.getPose().distanceFrom(lockdownPose); // Assumes getDistance exists
                // If getDistance doesn't exist, use: Math.hypot(follower.getPose().getX() - lockedPose.getX(), follower.getPose().getY() - lockedPose.getY());

                double headingError = Math.abs(follower.getPose().getHeading() - lockdownPose.getHeading());
                // Normalize heading error to 0-360 or 0-180 if needed here

                // --- TOLERANCE CHECK (Anti-Buzz) ---
                if (distanceError < HOLD_TOLERANCE_INCHES && headingError < Math.toRadians(HOLD_TOLERANCE_DEGREES)) {
                    // We are close enough. Relax the motors to stop buzzing.
                    follower.breakFollowing();
                    // Optional: explicitly set 0 power if breakFollowing doesn't coast
                    // follower.setTeleOpMovement(0, 0, 0, false);

                    if (timingSystem.do_telemetry()) {
                        telemetry.addData("Status", "HOLDING (Relaxed - In Deadzone)");
                    }
                } else {
                    // We were pushed! FIGHT BACK.

                    // Create the path dynamically from CURRENT -> LOCKED
                    // This pulls the robot back to the lockedPose
                    Path lockdownPath = new Path(new BezierLine(follower.getPose(), lockdownPose));
                    lockdownPath.setConstantHeadingInterpolation(lockdownPose.getHeading());

                    follower.followPath(lockdownPath, true);
                    follower.update();

                    if (timingSystem.do_telemetry()) {
                        telemetry.addData("Status", "HOLDING (Correcting Error!)");
                    }
                }

                if (timingSystem.do_telemetry()) {
                    telemetry.addData("Err Dist", "%.2f in", distanceError);
                }
            }
        }

//        if (holdingPosition) {
//            // --- ENGAGE HOLD (THE FIX) ---
//
//            // 1. Capture exact current pose
//            Pose currentPose = follower.getPose();
//
//            // 2. Create a "Path" that starts and ends at the exact same spot.
//            //    This forces the Follower to enter "Path Following" mode.
//            Point holdPoint = new Point(currentPose);
//            Path holdPath = new Path(new BezierLine(holdPoint, holdPoint));
//
//            // 3. Set the heading to remain constant (the current heading)
//            holdPath.setConstantHeadingInterpolation(currentPose.getHeading());
//
//            // 4. Instruct Follower to follow this path and HOLD the end
//            follower.followPath(holdPath, true);
//
//        } else {
//            // --- RELEASE HOLD ---
//            // Break the path following to return to TeleOp control
//            follower.breakFollowing();
//            follower.startTeleOp();
//        }


//        follower.activateHeading();
//        forwards = new Path(new BezierLine(new Pose(0,0), new Pose(DISTANCE,0)));
//        forwards.setConstantHeadingInterpolation(0);
//        backwards = new Path(new BezierLine(new Pose(DISTANCE,0), new Pose(0,0)));
//        backwards.setConstantHeadingInterpolation(0);
//        follower.followPath(forwards);

        //Stop automated following if the follower is done or we want to cancel.
        if (automatedDrive &&
//                (Math.abs(gamepad1.left_stick_x)>0.5) ||
//                (Math.abs(gamepad1.left_stick_y)>0.5) ||
                (gamepad1.xWasPressed() ||
                        (!follower.isBusy() && !lockdownMode))) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }


        //Moderate speed mode
//        if (gamepad1.leftBumperWasPressed()) {
//            slowMode = !slowMode;
//        }
        if (gamepad1.leftBumperWasPressed()) {
            if (driveSpeedMax == fullSpeed) {
                // Toggle to moderateSpeed
                driveSpeedMax = fullSpeed; // fullSpeed or moderateSpeed
                follower.setMaxPower(driveSpeedMax);
            } else {
                // Toggle back to fullSpeed
                driveSpeedMax = moderateSpeed; // fullSpeed or moderateSpeed
                follower.setMaxPower(driveSpeedMax);
            }
        }


        //Slow Mode
//        if (gamepad1.rightBumperWasPressed()) {
//            slowMode = !slowMode;
//        }
        if (gamepad1.right_bumper) {
            slowMode = true;
        } else {
            slowMode = false;
        }

//        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }

//        greenLED = hardwareMap.get(DigitalChannel.class, "lockdown_LED1");
//        redLED = hardwareMap.get(DigitalChannel.class, "lockdown_LED2");

        if (automatedDrive==true) {
            redLED.enable(true);
        } else {
            redLED.enable(false);
        }

        //Call this once per loop
        follower.update();

        if (timingSystem.do_telemetry()) {
//            telemetryM.debug("position", follower.getPose());
//            telemetryM.debug("velocity", follower.getVelocity());
//            telemetryM.debug("automatedDrive", automatedDrive);
//            telemetryM.update();

            this.sorter.balls.detailedTelemetry();
            this.sorter.balls.telemetry();
            telemetry.update();
        }
    }
}
