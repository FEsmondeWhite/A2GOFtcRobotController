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
    private FastColorSensor in_Sensor, hold_Sensor, out_Sensor;

    // SINGLE SOURCE OF TRUTH
    private Character[] inventory = {'N', 'N', 'N'};

    // Persistent survey object to prevent Garbage Collection (GC) spikes
    private final BallColorSurvey persistentSurvey = new BallColorSurvey(5);
    private boolean surveyInProgress = false;

    // Reusable list to avoid frequent allocations in update hardware loop
    private final List<Character> reusableColorList = Arrays.asList('N', 'N', 'N');

    private List<Character> patternList;
    private int patternIndex = 0;
    private Telemetry.Item ballStatusItem;

    public BallColors(@NonNull HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init() {
        // Your FastColorSensor handling code remains intact
        in_Sensor = new FastColorSensor(getI2cClient("Color_Sensor_in"));
        hold_Sensor = new FastColorSensor(getI2cClient("Color_Sensor_hold"));
        out_Sensor = new FastColorSensor(getI2cClient("Color_Sensor_out"));

        in_Sensor.setProximityThreshold(100);
        hold_Sensor.setProximityThreshold(110);
        out_Sensor.setProximityThreshold(125);

        ballStatusItem = telemetry.addData("Inventory", "In:%c Hold:%c Out:%c", inventory[0], inventory[1], inventory[2]);
        ballStatusItem.setRetained(true);
    }

    public void forceInitialInventory() {
        inventory[0] = 'P';
        inventory[1] = 'G';
        inventory[2] = 'P';
    }
    // --- SURVEY LOGIC ---

    /**
     * Resets the persistent survey object and starts a new sampling run.
     * @param samples Number of samples to take (e.g., 5 for start, 3 for quick refresh).
     */
    public void requestSurvey(int samples) {
        // Pass the existing inventory as the seed
        persistentSurvey.reset(samples, this.inventory);
        surveyInProgress = true;
    }

    // Ensure update() uses the new logic
    public void update() {
        if (surveyInProgress) {
            updateHardware();
            persistentSurvey.addSample(getHardwareColors());

            if (persistentSurvey.isComplete()) {
                Character[] results = persistentSurvey.getResult();
                // Use System.arraycopy for clean value transfer
                System.arraycopy(results, 0, inventory, 0, 3);
                surveyInProgress = false;
            }
        }
        telemetry();
    }

    private void updateHardware() {
        in_Sensor.update();
        hold_Sensor.update();
        out_Sensor.update();
    }

    /**
     * Optimized to update a reusable list instead of creating a new one.
     */
    private List<Character> getHardwareColors() {
        reusableColorList.set(0, in_Sensor.getBallChar());
        reusableColorList.set(1, hold_Sensor.getBallChar());
        reusableColorList.set(2, out_Sensor.getBallChar());
        return reusableColorList;
    }

    // --- VIRTUAL INVENTORY MANIPULATION ---

    public void shift() {
        // Standard carousel rotation: 0->1, 1->2, 2->0
        Character temp = inventory[2];
        inventory[2] = inventory[1]; // Hold -> Out
        inventory[1] = inventory[0]; // In -> Hold
        inventory[0] = temp; // Out -> In
    }

    public void launch() { inventory[2] = 'N'; }

    public Character getSlot(int i) { return inventory[i]; }

    public boolean isSurveying() { return surveyInProgress; }
    public Character[] getInventory() { return inventory; }
    // --- PATTERN LOGIC ---

    public void setPatternList(List<Character> list) { this.patternList = list; }
    public void incrementPatternIndex() { patternIndex++; }
    public void resetPatternIndex() { patternIndex = 0; }

    public Character targetPatternColor() {
        if (patternList == null || patternList.isEmpty()) return 'P';

        // Modulo operator ensures the index wraps (0, 1, 2, 0, 1, 2...)
        int wrapIndex = patternIndex % patternList.size();
        return patternList.get(wrapIndex);
    }

    // --- LOGICAL CHECKS ---

    /**
     * Checks the virtual inventory to see if the robot is carrying any balls at all.
     * @return true if at least one slot (In, Hold, or Out) has a ball.
     */
    public boolean anyBallAvailable() {
        return inventory[0] != 'N' || inventory[1] != 'N' || inventory[2] != 'N';
    }
    /**
     * Checks if the target color is specifically in the 'In' or 'Hold' slots.
     * If this is true, we know we should SORT to bring it to the 'Out' slot.
     */
    public boolean targetBallAvailableElsewhere() {
        char target = targetPatternColor();
        return inventory[0] == target || inventory[1] == target;
    }
    public boolean anyBallReadyForLaunch() { return inventory[2] != 'N'; }
    public boolean targetMatchForLaunch() { return inventory[2] == targetPatternColor(); }

    // --- HARDWARE CLIENT ---

    private I2cDeviceSynch getI2cClient(String name) {
        HardwareDevice device = hwMap.get(name);
        I2cDeviceSynchSimple simpleClient = (device instanceof I2cDeviceSynchDevice) ?
                ((I2cDeviceSynchDevice<?>) device).getDeviceClient() : hwMap.get(I2cDeviceSynchSimple.class, name);
        return new I2cDeviceSynchImplOnSimple(simpleClient, true);
    }

    public void telemetry() {
        ballStatusItem.setValue(String.format("In:%c Hold:%c Out:%c", inventory[0], inventory[1], inventory[2]));
    }
}