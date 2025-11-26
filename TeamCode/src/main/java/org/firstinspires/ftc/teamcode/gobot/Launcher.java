package org.firstinspires.ftc.teamcode.gobot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoController;

@Configurable
public class Launcher {



    ServoController ExpansionHub2_ServoController;
    private DcMotorEx motor;
    // 312 RPM @ no load (5.2) RPS)
    //  	537.7 PPR at output. (2796.04 PPS)
    double PPR =  	537.7*312/6000;
    private double TPS;
    private double RPS;
    private double nominalRPS;
    public static double actual_RPS = 0;



    public static double launcher_P;
    public static double launcher_I;
    public static double launcher_D;
    public static double launcher_F;

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
        return;
    }

    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "Flywheel");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // or FLOAT
        motor.setDirection(DcMotor.Direction.REVERSE);
        nominalRPS = 60;
        ExpansionHub2_ServoController = hwMap.get(ServoController.class, "Expansion Hub 2");
        ExpansionHub2_ServoController.pwmEnable();

//        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
//        PIDFCoefficients pidfOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(launcher_P, launcher_I, launcher_D, launcher_F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
    }

    public void update() {

//        // Re-read coefficients and verify change.
//        PIDFCoefficients pidfModified = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

//        telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
//                pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
//        telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
//                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

        actual_RPS = this.motor.getVelocity()/this.PPR;
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

}
