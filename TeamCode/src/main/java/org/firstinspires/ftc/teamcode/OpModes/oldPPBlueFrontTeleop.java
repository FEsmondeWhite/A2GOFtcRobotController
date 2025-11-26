package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gobot.Intake;
import org.firstinspires.ftc.teamcode.gobot.Launcher;
import org.firstinspires.ftc.teamcode.gobot.Lifter;
import org.firstinspires.ftc.teamcode.gobot.PoseStorage;
import org.firstinspires.ftc.teamcode.gobot.Sorter;

import java.util.function.Supplier;

@Disabled
@TeleOp(group = "TEST",name="Test: Blue front TeleOp PedroPath")
public class oldPPBlueFrontTeleop extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChainFront;
    private Supplier<PathChain> pathChainRear;
    private Supplier<PathChain> pathChainPark;

    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private int AllianceColor = 1; // Blue is 1
//    private int AllianceColor = 2; // Red is 2
    private int StartPosition = 1; // Front is 1
    //    private int StartPosition = 1; // Back is 2

    public static Pose startingPose;  //See ExampleAuto to understand how to use this

    private Pose frontShootPose;
    private Pose rearShootPose;
    private Pose ParkPose;

    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    private Launcher launcher;

    public Pose StartingPose(int AllianceColor, int StartPosition) {
        if (AllianceColor == 1) {
            if (StartPosition == 1) {
                return new Pose(57.5, 9, Math.toRadians(90));
            } else if (StartPosition == 2) {
                return new Pose(29.5, 134, Math.toRadians(270));
            }
        } else if (AllianceColor == 2) {
            if (StartPosition == 1) {
                return new Pose(144 - 57.5, 9, Math.toRadians(90));
            } else if (StartPosition == 2) {
                return new Pose(144 - 29.5, 134, Math.toRadians(270));
            }
        }
        return null;
    }

    @Override
    public void init() {
        intake = new Intake();
        intake.init(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        sorter.init();
        lifter = new Lifter();
        lifter.init(hardwareMap);
        launcher = new Launcher();
        launcher.init(hardwareMap);

        if (AllianceColor == 1) {
            frontShootPose = new Pose(71,20,Math.toRadians(121));
            rearShootPose = new Pose(71,79, Math.toRadians(138));
            ParkPose = new Pose(144-38.6,33.4, Math.toRadians(90));
        } else if (AllianceColor == 2) {
            frontShootPose = new Pose(144-71,20,Math.toRadians(180-121));
            rearShootPose = new Pose(144-71,79, Math.toRadians(180-138));
            ParkPose = new Pose(38.6,33.4, Math.toRadians(90));
        }

        //Get pose from storage and apply heading offset based on alliance
        if(PoseStorage.currentPose==null) {
            // Create the new starting pose based on alliance and position
            startingPose = StartingPose(AllianceColor, StartPosition);
//            startingPose = startPose.getPose();

        } else {
            // This will load the pose if we have already run the robot.
            startingPose=PoseStorage.currentPose;
        }

        //Drivetrain (PP)
        follower=org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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
        //Call this once per loop
        follower.update();
        telemetryM.update();
        sorter.update();
        lifter.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        // Intake
        if (gamepad1.a) {
            intake.setIntake(2);
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
            launcher.setNominalRPS(60);
        }
        if (gamepad2.dpadLeftWasReleased())
        {
            launcher.setNominalRPS(55);
        }

        // static public flywheel PIDF allows it to be changed in panels
        telemetryM.addData("Flywheel setpoint", launcher.getNominalRPS());
        telemetryM.addData("Flywheel actual", launcher.getActualRPS());


        if (gamepad2.x) {
            launcher.enableMotor();
            telemetry.addData("Flywheel setpoint: ", launcher.getNominalRPS());
            telemetry.addData("Flywheel actual: ", launcher.getActualRPS());

        } else {
            launcher.disableMotor();
        }

        //Automated PathFollowing
        if (gamepad1.dpadDownWasPressed()) {
            follower.followPath(this.pathChainFront.get(), true);
            automatedDrive = true;
        }

        //Automated PathFollowing
        if (gamepad1.dpadUpWasPressed()) {
            follower.followPath(this.pathChainRear.get(), true);
            automatedDrive = true;
        }

        //Automated PathFollowing
        if (gamepad1.dpadLeftWasPressed()) {
            follower.followPath(this.pathChainPark.get(), true);
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.xWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
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

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.update();
    }
}
