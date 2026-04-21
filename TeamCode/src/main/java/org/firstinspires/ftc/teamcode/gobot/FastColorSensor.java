package org.firstinspires.ftc.teamcode.gobot;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer;

@I2cDeviceType
@DeviceProperties(name = "FastColorSensor", description = "Bulk-read driver for Rev V3 (APDS-9151)", xmlTag = "FastColorSensor")
public class FastColorSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
// This is a super-fast class for reading from RevRobotics ColorSensorV3 sensors.
    // APDS-9151 Registers
    private static final int REG_MAIN_CTRL = 0x00;
    private static final int REG_RES_RATE  = 0x02;
    private static final int REG_DATA_START = 0x08;
    private static final int BULK_READ_LENGTH = 14;

    private int red, green, blue, ir, proximity;
    private int proximityThreshold = 100; // Default

    public FastColorSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x52));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        // Enable RGB + Proximity + Active Mode
        deviceClient.write8(REG_MAIN_CTRL, 0x07);

        // 18-bit resolution (high precision for purple/green differentiation)
        // 0x22 = 100ms cycle time
        deviceClient.write8(REG_RES_RATE, 0x22);

        // Set REPEAT window to allow the Hub to fetch data in parallel across buses
        deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(
                REG_DATA_START, BULK_READ_LENGTH, I2cDeviceSynch.ReadMode.REPEAT));

        return true;
    }

    public void update() {
        // Pulls 14 bytes from the Hub's internal I2C cache
        byte[] buffer = deviceClient.read(REG_DATA_START, BULK_READ_LENGTH);

        // Proximity (2 bytes)
        proximity = (buffer[0] & 0xFF) | ((buffer[1] & 0xFF) << 8);

        // IR (3 bytes)
        ir = (buffer[2] & 0xFF) | ((buffer[3] & 0xFF) << 8) | ((buffer[4] & 0xFF) << 16);

        // RGB (3 bytes each, Little Endian)
        green = (buffer[5] & 0xFF) | ((buffer[6] & 0xFF) << 8) | ((buffer[7] & 0xFF) << 16);
        red   = (buffer[8] & 0xFF) | ((buffer[9] & 0xFF) << 8) | ((buffer[10] & 0xFF) << 16);
        blue  = (buffer[11] & 0xFF) | ((buffer[12] & 0xFF) << 8) | ((buffer[13] & 0xFF) << 16);
    }

    // Getters for the sorter logic
    public int red()   { return red; }
    public int green() { return green; }
    public int blue()  { return blue; }
    public int getProximity() { return proximity; }
    public int getIr() { return ir; }

    public void setProximityThreshold(int threshold) {
        this.proximityThreshold = threshold;
    }

    public char getBallChar() {
        // Step 1: Check if a ball is present based on the specific sensor's threshold
        if (proximity < proximityThreshold) {
            return 'N';
        }

        // Step 2: Use ratios to classify
        // Use double casting to avoid integer division issues
        double r = Math.max(red, 1);   // Prevent division by zero
        double g = (double) green;
        double b = (double) blue;

        if (g / r > 1.5) {
            return 'G';
        } else if (b / r > 0.6) {
            return 'P';
        }

        return 'U'; // 'U' for Unknown/Uncertain
    }

    @Override
    public com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer getManufacturer() {
        return com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        // This is the string that shows up in logs or telemetry
        // if you print the object directly
        return "Fast Rev Color Sensor V3";
    }

    @Override
    public String getConnectionInfo() {
        // Returns the I2C bus and address info
        return deviceClient.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        // Standard versioning for the driver
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        // Optional: clear settings between OpModes if needed
    }

    @Override
    public void close() {
        deviceClient.close();
    }
}
