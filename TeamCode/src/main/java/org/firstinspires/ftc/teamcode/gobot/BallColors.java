package org.firstinspires.ftc.teamcode.gobot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Arrays;
import java.util.List;

public class BallColors {
    private HardwareMap hwMap;
    private Telemetry telemetry;

    // The three optimized sensors
    private FastColorSensor in_Sensor, hold_Sensor, out_Sensor;

    private List<Character> patternList;
    private int patternIndex = 0;
    private Telemetry.Item ballStatusItem;
    private String lastStatus = "";

    public BallColors(@NonNull HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init() {
        // Initialize sensors using the optimized I2C wrapper
        in_Sensor = new FastColorSensor(getI2cClient("Color_Sensor_in"));
        hold_Sensor = new FastColorSensor(getI2cClient("Color_Sensor_hold"));
        out_Sensor = new FastColorSensor(getI2cClient("Color_Sensor_out"));

        // Set proximity thresholds based on your test results
        in_Sensor.setProximityThreshold(100);
        hold_Sensor.setProximityThreshold(110);
        out_Sensor.setProximityThreshold(125);

        // Persistent telemetry line
        ballStatusItem = telemetry.addData("Ball Colors", "In: N  Hold: N  Out: N");
        ballStatusItem.setRetained(true);
    }

    private I2cDeviceSynch getI2cClient(String name) {
        HardwareDevice device = hwMap.get(name);
        I2cDeviceSynchSimple simpleClient;
        if (device instanceof I2cDeviceSynchDevice) {
            simpleClient = ((I2cDeviceSynchDevice<?>) device).getDeviceClient();
        } else {
            simpleClient = hwMap.get(I2cDeviceSynchSimple.class, name);
        }
        return new I2cDeviceSynchImplOnSimple(simpleClient, true);
    }

    public void updateColors() {
        // Trigger the bulk reads for all three sensors
        in_Sensor.update();
        hold_Sensor.update();
        out_Sensor.update();

        telemetry(); // Update the telemetry line if state changed
    }

    public List<Character> getColorList() {
        return Arrays.asList(in_Sensor.getBallChar(), hold_Sensor.getBallChar(), out_Sensor.getBallChar());
    }

    // --- Pattern Logic ---

    public void setPatternList(List<Character> patternList) { this.patternList = patternList; }
    public List<Character> getPatternList() { return patternList; }
    // Add this method
    public int getPatternIndex() {
        return this.patternIndex;
    }

    // Ensure your increment logic is there too
    public void incrementPatternIndex() {
        patternIndex++;
    }
    public void resetPatternIndex() { this.patternIndex = 0; }
    public boolean validPatternIndex() { return (patternIndex >= 0 && patternIndex < patternList.size()); }

    public Character targetPatternColor() {
        if (patternList != null && validPatternIndex()) {
            return patternList.get(patternIndex);
        }
        return 'E'; // Error
    }

    // --- Logical Checks ---

    public boolean anyBallReadyForLaunch() {
        return (out_Sensor.getBallChar() != 'N');
    }

    public boolean anyBallAvailable() {
        return (in_Sensor.getBallChar() != 'N' || hold_Sensor.getBallChar() != 'N' || out_Sensor.getBallChar() != 'N');
    }

    public boolean targetColorAvailable() {
        return getColorList().contains(targetPatternColor());
    }

    public boolean targetMatchForLaunch() {
        return (out_Sensor.getBallChar() == targetPatternColor());
    }

    // --- Telemetry ---

    public void telemetry() {
        char in = in_Sensor.getBallChar();
        char hold = hold_Sensor.getBallChar();
        char out = out_Sensor.getBallChar();

        String currentStatus = String.format("In: %c  Hold: %c  Out: %c", in, hold, out);
        if (!currentStatus.equals(lastStatus)) {
            ballStatusItem.setValue(currentStatus);
            lastStatus = currentStatus;
        }
    }

    public void detailedTelemetry() {
        telemetry.addData("In Details", "P:%d R:%d G:%d", in_Sensor.getProximity(), in_Sensor.red(), in_Sensor.green());
        telemetry.addData("Hold Details", "P:%d R:%d G:%d", hold_Sensor.getProximity(), hold_Sensor.red(), hold_Sensor.green());
        telemetry.addData("Out Details", "P:%d R:%d G:%d", out_Sensor.getProximity(), out_Sensor.red(), out_Sensor.green());
    }
}