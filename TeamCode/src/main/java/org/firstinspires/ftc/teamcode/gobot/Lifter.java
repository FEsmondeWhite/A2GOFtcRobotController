package org.firstinspires.ftc.teamcode.gobot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class Lifter {

    private Servo lifter;
    private double liftPosition;
    private static final double bottomPosition = 1;
    private static final double topPosition = 0;

    ServoController ExpansionHub2_ServoController;

    private int lifterState;
    private long lifter_timer_start;


    /**
     * Initialize the lifter.
     */
    public void init(HardwareMap hwMap) {
        this.ExpansionHub2_ServoController = hwMap.get(ServoController .class, "Expansion Hub 2");
        this.lifter = hwMap.get(Servo.class, "lifter");
        this.lifterState = 0;
        this.lifter.scaleRange(0.38, 0.585);
        this.liftPosition = bottomPosition;
        this.lifter.setPosition(liftPosition);
        this.ExpansionHub2_ServoController.pwmEnable();
    }

    // Start the lifter.
    // Make sure the sorter is in the correct position for lifting the ball.
    public void start() {
        lifterState = 1;
        this.liftPosition = topPosition;
        this.lifter.setPosition(liftPosition);
        lifter_timer_start  = System.currentTimeMillis();
        return;
    }

    // Immediately stop the lifter by moving back to the home position
    public void stop() {
        if ((lifterState < 10) && (lifterState!=0)) {
            lifter_timer_start  = System.currentTimeMillis();
            lifterState = 10;
            this.liftPosition = bottomPosition;
            this.lifter.setPosition(liftPosition);
        }
        return;
    }

    public boolean isBusy() {
        return (lifterState != 0);
    }

    /**
     * update runs the lifter state machine
     */
    public void update() {
        // Make sure the sorter is in the correct position for lifting the ball.

        if (this.lifterState == 0) {
            // Do nothing
        } else if (lifterState == 1) {
            if (System.currentTimeMillis() - lifter_timer_start >= 375) {
                lifter_timer_start = System.currentTimeMillis();
                this.liftPosition = bottomPosition;
                this.lifter.setPosition(liftPosition);
                lifterState = 2;
            }
        } else if (lifterState == 2) {
            if (System.currentTimeMillis() - lifter_timer_start >= 375) {
                lifterState = 0;
            }
        } else if (lifterState == 10) {
            if (System.currentTimeMillis() - lifter_timer_start >= 375) {
                lifterState = 0;
            }
        }
    }
}
