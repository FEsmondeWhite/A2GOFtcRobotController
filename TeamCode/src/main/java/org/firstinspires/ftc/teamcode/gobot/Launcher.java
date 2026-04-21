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

    // 537.7 PPR at output.
    private final double PPR = 537.7 * 312.0 / 6000.0;

    private double nominalRPS = 60;      // The desired speed (setpoint)
    private double activeSetpointRPS = 0; // The current command (nominal or 0)
    public double actual_RPS = 0;        // The measured speed
    public static final double threshold = 1;

    private long lastReadTime = 0;
    private final long READ_INTERVAL_MS = 250; // Throttle hardware read to 4Hz

    LED greenLED;
    private boolean lastGreenState = false;

    private List<DataPoint> data;

    public double launcher_P = 25;
    public double launcher_I = 0.015;
    public double launcher_D = 0.0003;
    public double launcher_F = 14;

    static class DataPoint {
        double distance;
        double speed;

        public DataPoint(double distance, double speed) {
            this.distance = distance;
            this.speed = speed;
        }
    }

    public Launcher() {
        this.activeSetpointRPS = 0;
        this.nominalRPS = 60;
    }

    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "Flywheel");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotor.Direction.REVERSE);

        ExpansionHub2_ServoController = hwMap.get(ServoController.class, "Expansion Hub 2");
        ExpansionHub2_ServoController.pwmEnable();

        greenLED = hwMap.get(LED.class, "lockdown_LED2");
        greenLED.enable(false);

        PIDFCoefficients pidfNew = new PIDFCoefficients(launcher_P, launcher_I, launcher_D, launcher_F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        data = new ArrayList<>();
        data.add(new DataPoint(66.0, 55.0));
        data.add(new DataPoint(72.0, 53.0));
        data.add(new DataPoint(78.0, 55.0));
        data.add(new DataPoint(84.0, 55.0));
        data.add(new DataPoint(90.0, 56.0));
        data.add(new DataPoint(100.0, 56));
        data.add(new DataPoint(141.0, 57.0));

//Distance	Flywheel Speed
//66	55
//72	53
//78	53
//84	53
//90	52
//100	52.5
//141	57

    }

    public void update() {
        long currentTime = System.currentTimeMillis();

        // 1. Throttled Hardware Read (Significant Loop Speed Gain)
        if (currentTime - lastReadTime > READ_INTERVAL_MS) {
            actual_RPS = motor.getVelocity() / PPR;
            lastReadTime = currentTime;
        }

        // 2. LED State Caching (Bus Silence)
        boolean shouldBeGreen = (activeSetpointRPS > 0) && isReady();
        if (shouldBeGreen != lastGreenState) {
            greenLED.enable(shouldBeGreen);
            lastGreenState = shouldBeGreen;
        }
    }

    /**
     * Updates the nominal speed and immediately applies it if the motor is currently enabled.
     */
    public void setNominalRPS(double setpoint) {
        this.nominalRPS = setpoint;
        // If motor is currently running, update the hardware velocity now
        if (this.activeSetpointRPS > 0) {
            this.activeSetpointRPS = nominalRPS;
            motor.setVelocity(activeSetpointRPS * PPR);
        }
    }

    public double getNominalRPS() {
        return this.nominalRPS;
    }
    public double getActiveSetpointRPS() {
        return this.activeSetpointRPS;
    }

    public void enableMotor() {
        this.activeSetpointRPS = nominalRPS;
        motor.setVelocity(activeSetpointRPS * PPR);
    }

    public void disableMotor() {
        this.activeSetpointRPS = 0;
        motor.setVelocity(0);
    }

    public boolean isReady() {
        // We compare against activeSetpointRPS so the LED doesn't stay green when motor is off
        return (activeSetpointRPS > 0) && (Math.abs(actual_RPS - activeSetpointRPS) < threshold);
    }

    public double getSpeedNearestToDistance(double targetDistance) {
        if (data == null || data.isEmpty()) return -1;

        double nearestSpeed = -1;
        double minDiff = Double.MAX_VALUE;

        for (DataPoint dp : data) {
            double diff = Math.abs(dp.distance - targetDistance);
            if (diff < minDiff) {
                minDiff = diff;
                nearestSpeed = dp.speed;
            }
        }
        return nearestSpeed;
    }

    public double getDistance(Pose robotPosition, Pose goalPosition) {
        return robotPosition.distanceFrom(goalPosition);
    }
}