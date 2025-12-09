package org.firstinspires.ftc.teamcode.gobot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class EnPointe {

    private Servo servo;
    private double position;
    private static final double normalPosition = 1;
    private static final double liftedPosition = 0;

    ServoController ExpansionHub2_ServoController;

    private int enPointState;
    private long enpointe_timer_start;


    /**
     * Initialize the lifter.
     */
    public void init(HardwareMap hwMap) {
        this.ExpansionHub2_ServoController = hwMap.get(ServoController .class, "Expansion Hub 2");
        this.servo = hwMap.get(Servo.class, "En Pointe");
        this.enPointState = 0;
        this.servo.scaleRange(0.29, 0.63);
        this.position = normalPosition;
        this.servo.setPosition(position);
        this.ExpansionHub2_ServoController.pwmEnable();
    }

    // Start the lifter.
    // Make sure the sorter is in the correct position for lifting the ball.
    public void start() {
        if ((enPointState==0)||(enPointState==10)) {
            enPointState = 1;
            this.position = liftedPosition;
            this.servo.setPosition(position);
            enpointe_timer_start = System.currentTimeMillis();
        }
    }

    // Immediately stop the lifter by moving back to the home position
    public void stop() {
        if (enPointState!=0) {
            enpointe_timer_start = System.currentTimeMillis();
            enPointState = 10;
            this.position = normalPosition;
            this.servo.setPosition(position);
        }
    }

    public boolean isBusy() {
        return (enPointState != 0);
    }

    /**
     * update runs the lifter state machine
     */
    public void update() {
        // Make sure the sorter is in the correct position for lifting the ball.

        if (this.enPointState == 0) {
            // Do nothing
        } else if (enPointState == 1) {
            // We're raising up
            if (System.currentTimeMillis() - enpointe_timer_start >= 2500) { // Still lifting if time < 2500
                enPointState = 2;
            }
        } else if (enPointState == 2) {
            // We're in the lifted state
            // Do nothing
        } else if (enPointState == 10) {
            // We're lowering down
            if (System.currentTimeMillis() - enpointe_timer_start >= 1000) {
                enPointState = 0;
            }
        }
    }

}
