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

    private HardwareMap hwMap;
    private Telemetry telemetry;

    private CRServoImplEx sorterServo;
    // GoBilda dual-mode torque servo
    //    ExpansionHub2_ServoController = hardwareMap.get(ServoController.class, "Expansion Hub 2");
    public BallColors balls;

//    private int sorterState; // for sorter state machine
//    long sorterTimerStart;
    private ElapsedTime sorterTimer;
    TouchSensor sorterAngleMagnet;
//    TouchSensor sorterPositionMagnetA;
//    TouchSensor sorterPositionMagnetB;
    boolean sorterMagnetIsTriggered;
//    boolean sorterPositionATriggered;
//    boolean sorterPositionBTriggered;
    int sorterDirection;

    public double fastMovePower = 1.0;
    public double fastTime = 0.35;
    public double slowMovePower = 0.13;
    public double slowTime = 0.05;
    public double additionalTime = 0.04;
    public double holdPower = 0;
    public double holdTime = 0.1;
    public double stopPower = 0;
    public double stopTime = 0.05;

    enum SorterState {
        SORTER_IDLE,      // not moving
        START_SORTING, // start the sorter
        FAST_MOVE, // start moving quickly
        SLOW_MOVE, // slowly finish the move
        BRIEF_SLOW_MOVE, // briefly add a touch more rotation
        BRIEF_HOLD, // hold at position to allow the sorter to settle
        STOP_FOR_IDLE // unpower servo to be ready for idle
    }
    private SorterState sorterState;

    public Sorter (@NonNull HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init() {
        this.sorterServo = hwMap.get(CRServoImplEx.class, "ball_sorter_arm");
        this.sorterAngleMagnet = hwMap.get(TouchSensor.class, "selector_paddle_magnet");
        balls = new BallColors(hwMap, this.telemetry);
        this.balls.init();
        this.balls.updateColors();
        sorterState = SorterState.SORTER_IDLE;
        sorterTimer = new ElapsedTime();
        this.stop();
    }

    public void stop() {
        this.sorterServo.setPower(stopPower);
        this.sorterServo.setPwmDisable();
    }

    public void updateMagnets() {
        this.sorterMagnetIsTriggered = this.sorterAngleMagnet.isPressed();
//        this.sorterPositionATriggered = this.sorterPositionMagnetA.isPressed();
//        this.sorterPositionBTriggered = this.sorterPositionMagnetB.isPressed();
    }

    // start the sorter to move counter-clockwise or cw.
    // The mechanism only works properly in the ccw direction (direction = 1)
    public void start(int direction) {
        if (this.sorterState == SorterState.SORTER_IDLE) {
            if (direction == 1) { // ccw
                this.sorterDirection = 1;
                this.sorterServo.setDirection(CRServo.Direction.FORWARD);
            }
            else if (direction == -1) { // cw
                this.sorterDirection = -1;
                this.sorterServo.setDirection(CRServo.Direction.REVERSE);
            }
            this.sorterState = SorterState.FAST_MOVE;
            this.sorterServo.setPwmEnable();
            this.sorterServo.setPower(fastMovePower);
            this.sorterTimer.reset(); //  = System.currentTimeMillis();
        }
        return;
    }

    public boolean isBusy() {
        return (this.sorterState != SorterState.SORTER_IDLE);
    }

    /**
     * update runs the sorter state machine.
     */
    public void update() {
        this.updateMagnets();

        switch (this.sorterState) {
            // Idle
            case SORTER_IDLE:
                break;
            // Start the sorting process
            case START_SORTING:
                this.sorterState = SorterState.FAST_MOVE;
//                this.sorterServo.setPwmEnable();
                this.sorterServo.setPower(fastMovePower);
                this.sorterTimer.reset();
                break;
            // Initial quick movement
            case FAST_MOVE:
                if (this.sorterTimer.time() > fastTime) {
                    this.sorterTimer.reset();
                    this.sorterState = SorterState.SLOW_MOVE;
                    this.sorterServo.setPower(slowMovePower);
                }
                break;
            // Secondary slow movement
            case SLOW_MOVE:
                if ((this.sorterTimer.time() > slowTime) && this.sorterMagnetIsTriggered) {
                    this.sorterTimer.reset();
                    this.sorterState = SorterState.BRIEF_SLOW_MOVE;
                }
                break;
            // Extra nudge
            case BRIEF_SLOW_MOVE:
                if (this.sorterTimer.time() > additionalTime) {
                    this.sorterTimer.reset();
                    this.sorterState = SorterState.BRIEF_HOLD;
                    this.sorterServo.setPower(holdPower);
                }
                break;
            // Hold period
            case BRIEF_HOLD:
                if (this.sorterTimer.time() > holdTime) {
//                    if  (this.sorterMagnetIsTriggered) {
                    this.sorterTimer.reset();
                    this.sorterState = SorterState.STOP_FOR_IDLE;
                    this.balls.updateColors();
//                        this.stop();
                    this.sorterServo.setPower(stopPower);
//                    } else {
//                        this.sorterState = SorterState.START_SORTING;
//                    }
                }
                break;
            // Stop for idle
            case STOP_FOR_IDLE:
                if (this.sorterTimer.time() > stopTime) {
                    this.sorterState = SorterState.SORTER_IDLE;
                }
                break;
        }
    }
}

