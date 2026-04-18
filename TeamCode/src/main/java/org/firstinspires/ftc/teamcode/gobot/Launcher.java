package org.firstinspires.ftc.teamcode.gobot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.ArrayList;
import java.util.List;
@Configurable
public class Launcher {



    private ServoController ExpansionHub2_ServoController;
    private DcMotorEx motor;
    // 312 RPM @ no load (5.2) RPS)
    //  	537.7 PPR at output. (2796.04 PPS)
    double PPR =  	537.7*312/6000;
    private double TPS;
    private double RPS;
    private double nominalRPS;
    public double actual_RPS = 0;
    public static final double threshold = 2;

    // Declare a LED object for the indicator LEDs
    LED greenLED;

    private List<DataPoint> data;


    public double launcher_P;
    public double launcher_I;
    public double launcher_D;
    public double launcher_F;

    // Custom class to hold distance and speed pairs
    static class DataPoint {
        double distance;
        double speed;

        public DataPoint(double distance, double speed) {
            this.distance = distance;
            this.speed = speed;
        }
    }

//    /**
//     * Finds the speed value corresponding to the distance nearest to a given target distance.
//     *
//     * @param dataPoints A list of DataPoint objects containing distance and speed.
//     * @param targetDistance The distance to find the nearest speed for.
//     * @return The speed value corresponding to the nearest distance, or -1 if the list is empty.
//     */
    public double getSpeedNearestToDistance(double targetDistance) {
        if (data == null || data.isEmpty()) {
            return -1; // Or throw an IllegalArgumentException
        }

        double nearestSpeed = -1;
        double minDistanceDifference = Double.MAX_VALUE;

        for (DataPoint dp : data) {
            double currentDifference = Math.abs(dp.distance - targetDistance);

            if (currentDifference < minDistanceDifference) {
                minDistanceDifference = currentDifference;
                nearestSpeed = dp.speed;
            }
        }
        return nearestSpeed;
    }

    public Launcher() {
        this.RPS = 0;
        this.TPS = 0;
        this.launcher_P = 25;
        this.launcher_I = 0.015;
        this.launcher_D = 0.0003;
        this.launcher_F = 14;
    }

    public double getNominalRPS() {
        return this.nominalRPS;
    }

    public double getActualRPS() {
        return this.motor.getVelocity()/this.PPR;
    }


    public void setNominalRPS(double setpoint) {
        nominalRPS = setpoint;
    }

    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "Flywheel");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // or FLOAT
        motor.setDirection(DcMotor.Direction.REVERSE);
        nominalRPS = 60;
        ExpansionHub2_ServoController = hwMap.get(ServoController.class, "Expansion Hub 2");
        ExpansionHub2_ServoController.pwmEnable();

        // turn green when the flywheel speed is within 1 rps of the target value
        greenLED = hwMap.get(LED.class, "lockdown_LED2");
        // Red is used to indicate automatic movement program is running.

        greenLED.enable(false);

//        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
//        PIDFCoefficients pidfOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(launcher_P, launcher_I, launcher_D, launcher_F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        data = new ArrayList<>();
        data.add(new DataPoint(66.0, 55.0));
        data.add(new DataPoint(72.0, 53.0));
        data.add(new DataPoint(78.0, 53.0));
        data.add(new DataPoint(84.0, 53.0));
        data.add(new DataPoint(90.0, 52.0));
        data.add(new DataPoint(100.0, 52.5));
        data.add(new DataPoint(141.0, 57.0));
    }
//Distance	Flywheel Speed
//66	55
//72	53
//78	53
//84	53
//90	52
//100	52.5
//141	57

    public void update() {

//        // Re-read coefficients and verify change.
//        PIDFCoefficients pidfModified = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

//        telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
//                pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
//        telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
//                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

        actual_RPS = this.motor.getVelocity()/this.PPR;

        // Enable Green LED if the launcher is running and at nominal speed
        if ((!(this.RPS > 0)) || (!isReady())) {
            greenLED.enable(false);
        } else {
            greenLED.enable(true);
        }
    }

    public boolean isReady() {
        return (Math.abs(actual_RPS - this.RPS) <threshold);
    }

    public void setMotorSpeed(double speed){
        motor.setPower(speed);
    }

    /**
     * Assign a new flywheel speed
     */
    public void enableMotor() {
        this.RPS = nominalRPS;
        this.TPS = (PPR*this.RPS);
        motor.setVelocity(this.TPS);
    }

    public void disableMotor() {
        this.RPS = 0;
        this.TPS = (PPR*this.RPS);
        motor.setVelocity(this.TPS);
    }

    public double getTPS() {
        return this.TPS;
    }

    public double getDistance(Pose robotPosition, Pose goalPosition) {
        double xDistance = robotPosition.getX() - goalPosition.getX();
        double yDistance = robotPosition.getY() - goalPosition.getY();
        return Math.sqrt(Math.pow(xDistance,2) + Math.pow(yDistance,2));
    }

//    double speed = getSpeedNearestToDistance(double targetDistance);


}
