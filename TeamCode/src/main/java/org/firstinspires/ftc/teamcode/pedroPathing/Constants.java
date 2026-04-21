package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.33) // 31.6 lbs -> 14.33 up from 13.245
            .forwardZeroPowerAcceleration(-33.412)
            .lateralZeroPowerAcceleration(-73.688)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.003, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.02, 0.03))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.12, 0.063, .001888));
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0.0,0.00001,0.6,0.02));
// robot mass is 29.2 lbs, converted to kg is 13.245

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(62.038)
            .yVelocity(49.284)
            .rightFrontMotorName("FrontRight")
            .rightRearMotorName("BackRight")
            .leftRearMotorName("BackLeft")
            .leftFrontMotorName("FrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(160/25.4) // 160 mm, converted to inches, 6.299. Measured as 6.375
            .strafePodX(-172/25.4) // -172, converted to inches, -6.7716. Measured as -6.7   Previously -146.5 mm
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
