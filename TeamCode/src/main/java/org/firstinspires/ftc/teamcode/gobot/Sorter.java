package org.firstinspires.ftc.teamcode.gobot;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Sorter {

    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    private CRServo ball_sorter_arm;
    // GoBilda dual-mode torque servo
    //    ExpansionHub2_ServoController = hardwareMap.get(ServoController.class, "Expansion Hub 2");
    public BallColors balls;

    private TouchSensor mag7center;
    private TouchSensor mag6center;
    private TouchSensor mag4lead;
    private TouchSensor mag5lag;

//    private int sorterState; // for sorter state machine
//    long sorterTimerStart;
    int sorter_state;
    long sorter_timer_start;

    public Sorter (@NonNull HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init() {

        ball_sorter_arm = hwMap.get(CRServo.class, "ball_sorter_arm");
        mag7center = hwMap.get(TouchSensor.class, "mag7center");
        mag6center = hwMap.get(TouchSensor.class, "mag6center");
        mag4lead = hwMap.get(TouchSensor.class, "mag4lead");
        mag5lag = hwMap.get(TouchSensor.class, "mag5lag");

        sorter_state = 0;
        ball_sorter_arm.setDirection(CRServo.Direction.FORWARD);

        balls = new BallColors(hwMap, this.telemetry);
        this.balls.init();
        this.balls.updateColors();
//        sorterTimer = new ElapsedTime();
//        this.stop();
    }

//    public void stop() {
//        this.sorterServo.setPower(stopPower);
//        this.sorterServo.setPwmDisable();
//    }
//
//    public void updateMagnets() {
//        this.sorterMagnetIsTriggered = this.sorterAngleMagnet.isPressed();
////        this.sorterPositionATriggered = this.sorterPositionMagnetA.isPressed();
////        this.sorterPositionBTriggered = this.sorterPositionMagnetB.isPressed();
//    }

    // start the sorter to move counter-clockwise or cw.
    // The mechanism only works properly in the ccw direction (direction = 1)
    public void start(int direction) {
        sorter_state = 1;
    }

    public boolean isBusy() {
        return (this.sorter_state != 0);
    }

    /**
     * update runs the sorter state machine.
     */
    public void update() {
        if (0 == sorter_state) {
            if (!(mag7center.isPressed() || mag6center.isPressed())) {
                ball_sorter_arm.setPower(-0.2);
                sorter_state = 10;
            }
        } else if (1 == sorter_state) {
            ball_sorter_arm.setPower(1);
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            sorter_timer_start = System.currentTimeMillis();
            sorter_state = 2;
        } else if (2 == sorter_state) {
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            if (System.currentTimeMillis() - sorter_timer_start > 430) {
                ball_sorter_arm.setPower(0.15);
                // Get the current time in milliseconds. The value returned represents
                // the number of milliseconds since midnight, January 1, 1970 UTC.
                sorter_timer_start = System.currentTimeMillis();
                sorter_state = 3;
            }
        } else if (3 == sorter_state) {
            if (mag4lead.isPressed()) {
                ball_sorter_arm.setPower(0);
                sorter_state = 0;
            }
        } else if (10 == sorter_state) {
            if (mag7center.isPressed() || mag6center.isPressed()) {
                ball_sorter_arm.setPower(0);
                sorter_state = 0;
            }
        }


    }
}



