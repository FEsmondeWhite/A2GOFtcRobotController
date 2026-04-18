package org.firstinspires.ftc.teamcode.gobot;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TimingOptimization {
    private ElapsedTime loopTimer;
    private ElapsedTime hzUpdateTimer;
    private Telemetry telemetry;

    private double lastHz;
    private double minHz;
    private double maxHz;
    private int startingCountdown;
    private boolean enableTelemetry = false;

    public TimingOptimization(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init() {
        this.loopTimer = new ElapsedTime();
        this.hzUpdateTimer = new ElapsedTime();
        this.minHz = 500;
        this.maxHz = 0;
        this.lastHz = 0;
        this.startingCountdown = 25; // ~ 0.5 seconds before timing begins to run.
    }

    public void update() {
        // 1. Measure the time ELAPSED since the end of the last loop
        double deltaTime = loopTimer.seconds();

        // 2. RESET immediately to start timing the NEXT loop
        loopTimer.reset();

        // 3. Calculate Hz based on that specific gap
        if (deltaTime > 0)  {
            double currentHz = 1.0 / deltaTime;

            if (startingCountdown == 0 ) {
                // Update Min/Max (Skip the first few loops to let JVM warm up)
                if (currentHz < this.minHz) this.minHz = currentHz;
                if (currentHz > this.maxHz) this.maxHz = currentHz;
            } else {
                startingCountdown--;
            }

            this.lastHz = currentHz;
        }

        // 4. Handle Telemetry Throttling
        if (this.hzUpdateTimer.seconds() > 0.25) {
            this.enableTelemetry = true;
            this.hzUpdateTimer.reset();
        } else {
            this.enableTelemetry = false;
        }

        // 5. Add data ONLY if telemetry is enabled for this frame
        if (this.enableTelemetry) {
            this.telemetry.addLine("--- Performance Monitor ---");
            this.telemetry.addData("Loop Speed (Hz)", "%3.1f", this.lastHz);
            this.telemetry.addData("Worst Case Hz", "%3.1f", this.minHz);
            this.telemetry.addData("Best Case Hz", "%3.1f", this.maxHz);
        }
    }

    public boolean do_telemetry() {
        return enableTelemetry;
    }
}