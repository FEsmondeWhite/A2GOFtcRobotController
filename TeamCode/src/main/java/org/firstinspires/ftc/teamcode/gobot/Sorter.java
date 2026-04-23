package org.firstinspires.ftc.teamcode.gobot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sorter {
    private final HardwareMap hwMap;
    private final Telemetry telemetry;
    private CRServoImplEx ball_sorter_arm;
    private Lifter lifter;

    public BallColors balls;
    private TouchSensor mag7center, mag6center, mag4lead, mag5lag;

    private int sorter_state = 0;
    private long sorter_timer_start;

    // --- FINAL STABLE PARAMETERS ---
    private final long BASE_BURST_MS = 300;
    private final double SEARCH_POWER = 0.20;
    private final long DWELL_DURATION_MS = 60;
    private final long UNBIND_DURATION = 90;

    public Sorter (@NonNull HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init(Lifter lifter) {
        this.lifter = lifter;
        ball_sorter_arm = (CRServoImplEx) hwMap.get(CRServo.class, "ball_sorter_arm");

        mag7center = hwMap.get(TouchSensor.class, "mag7center");
        mag6center = hwMap.get(TouchSensor.class, "mag6center");
        mag4lead = hwMap.get(TouchSensor.class, "mag4lead");
        mag5lag = hwMap.get(TouchSensor.class, "mag5lag");

        ball_sorter_arm.setDirection(CRServo.Direction.FORWARD);
        ball_sorter_arm.setPwmEnable();

        // Initialize the BallColors database
        balls = new BallColors(hwMap, this.telemetry);
        balls.init();

        // Initial "Best of 5" to see what's in the robot at start
        balls.requestSurvey(5);

        sorter_state = 0;
    }

    public void start(int direction) {
        if (lifter.isBusy()) return;
        if (sorter_state == 0) sorter_state = 1;
    }

    public void startUnbind() {
        if (lifter.isBusy()) return;
        sorter_state = 20;
    }

    public boolean isBusy() { return (sorter_state != 0); }

    private boolean isAnyMagPressed() {
        if (sorter_state == 3 || sorter_state == 11) {
            return mag7center.isPressed() || mag6center.isPressed() ||
                    mag4lead.isPressed() || mag5lag.isPressed();
        }
        return false;
    }

    public void update() {
        long currentTime = System.currentTimeMillis();

        // EMERGENCY STOP: Lifter Protection
        if (lifter.isBusy() && sorter_state != 0) {
            ball_sorter_arm.setPower(0);
            sorter_state = 0;
            return;
        }

        // Process color surveys only when stationary
        if (sorter_state == 0) {
            balls.update();
        }

        switch (sorter_state) {
            case 0: // IDLE
                break;

            case 1: // START BURST
                ball_sorter_arm.setPower(1.0);
                sorter_timer_start = currentTime;
                sorter_state = 2;
                break;

            case 2: // BURST WAIT
                if (currentTime - sorter_timer_start > BASE_BURST_MS) {
                    ball_sorter_arm.setPower(SEARCH_POWER);
                    sorter_state = 3;
                }
                break;

            case 3: // SEEKING
                if (isAnyMagPressed()) {
                    ball_sorter_arm.setPower(0);
                    sorter_timer_start = currentTime;
                    sorter_state = 4;
                }
                break;

            case 4: // SETTLE DWELL & DATA SHIFT
                if (currentTime - sorter_timer_start > DWELL_DURATION_MS) {
                    // Update the virtual database
                    balls.shift();
                    balls.requestSurvey(3); // Best-of-3 refresh to check new positions
                    sorter_state = 0;
                }
                break;

            case 10: // INITIAL HOME
                ball_sorter_arm.setPower(0.12);
                sorter_state = 11;
                break;

            case 11: // WAIT FOR HOME
                if (isAnyMagPressed()) {
                    ball_sorter_arm.setPower(0);
                    sorter_timer_start = currentTime;
                    sorter_state = 4;
                }
                break;

            case 20: // UNBIND
                ball_sorter_arm.setPower(-0.4);
                sorter_timer_start = currentTime;
                sorter_state = 21;
                break;

            case 21: // UNBIND WAIT
                if (currentTime - sorter_timer_start > UNBIND_DURATION) {
                    sorter_state = 1;
                }
                break;
        }
    }

    public void addTelemetry() {
        telemetry.addData("Sorter State", sorter_state);
        telemetry.addData("Survey Active", balls.isSurveying());
    }
}
