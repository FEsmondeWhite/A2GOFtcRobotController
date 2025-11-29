package org.firstinspires.ftc.teamcode.gobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class Intake {
    private DcMotorEx motor;
    // 117 RPM @ no load (1.95 RPS) // Slow intake motor
    // 1,425.1 PPR at output. (2778.945 PPS)
    // 312 RPM @ no load (5.2 RPS) // Fast intake motor
    // 537.7 PPR at output. (2,796.04 PPS)
    double PPR = 537.7;
    private double tps;
    private double rps;

    public Intake() {
        this.rps = 0;
        this.tps = 0;
    }

    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "Intake");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // or FLOAT
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMotorSpeed(double speed){
        motor.setPower(speed);
        }

    /**
     * Assign a new intake speed
     */
    public void setIntake(double intake_rps) {
        this.rps = intake_rps;
        this.tps = (PPR*this.rps);
        motor.setVelocity(this.tps);
    }

    public double getRPS() {
        return this.rps;
    }

    public double getTPS() {
        return this.tps;
    }

}
