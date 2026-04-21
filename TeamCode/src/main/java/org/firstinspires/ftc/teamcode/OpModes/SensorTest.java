package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.teamcode.gobot.FastColorSensor;

@TeleOp(name="Sensor Benchmark")
public class SensorTest extends OpMode {
    // This demonstrates the use of a super-fast class for reading from RevRobotics ColorSensorV3 sensors.

    FastColorSensor in_ColorSensor, hold_ColorSensor, out_ColorSensor;

//    FastColorSensor sorter1, sorter2, sorter3;

    @Override
    public void init() {
        // Helper to extract the client without triggering Generic capture errors
        in_ColorSensor = new FastColorSensor(getI2cClient("Color_Sensor_in"));
        hold_ColorSensor = new FastColorSensor(getI2cClient("Color_Sensor_hold"));
        out_ColorSensor = new FastColorSensor(getI2cClient("Color_Sensor_out"));

        in_ColorSensor.setProximityThreshold(100);
        hold_ColorSensor.setProximityThreshold(110);
        out_ColorSensor.setProximityThreshold(125);

        telemetry.addLine("All Sensors Initialized!");
    }

    private I2cDeviceSynch getI2cClient(String name) {
        HardwareDevice device = hardwareMap.get(name);
        I2cDeviceSynchSimple simpleClient;

        if (device instanceof I2cDeviceSynchDevice) {
            // This gets the LynxI2cDeviceSynchV2 client safely
            simpleClient = ((I2cDeviceSynchDevice<?>) device).getDeviceClient();
        } else {
            // If it's not already a SynchDevice, grab it directly
            simpleClient = hardwareMap.get(I2cDeviceSynchSimple.class, name);
        }

        // Wrap the simple client in a full Synch implementation
        // This provides the ReadWindow and heartbeat support our driver needs
        return new I2cDeviceSynchImplOnSimple(simpleClient, true);
    }

    // In: P>100 is a ball. G/R > 1.5 is green. B/R > 0.6 is purple.
    // Hold: P>110.
    // Out: P>125.

    @Override
    public void loop() {
        in_ColorSensor.update();
        hold_ColorSensor.update();
        out_ColorSensor.update();

        telemetry.addData("In  ", "[%c] P:%d IR:%d R:%d G:%d B:%d",
                in_ColorSensor.getBallChar(), in_ColorSensor.getProximity(), in_ColorSensor.getIr(),
                in_ColorSensor.red(), in_ColorSensor.green(), in_ColorSensor.blue());

        telemetry.addData("Hold", "[%c] P:%d IR:%d R:%d G:%d B:%d",
                hold_ColorSensor.getBallChar(), hold_ColorSensor.getProximity(), hold_ColorSensor.getIr(),
                hold_ColorSensor.red(), hold_ColorSensor.green(), hold_ColorSensor.blue());

        telemetry.addData("Out ", "[%c] P:%d IR:%d R:%d G:%d B:%d",
                out_ColorSensor.getBallChar(), out_ColorSensor.getProximity(), out_ColorSensor.getIr(),
                out_ColorSensor.red(), out_ColorSensor.green(), out_ColorSensor.blue());

        telemetry.update();
    }

//    @Override
//    public void init() {
//        // "color1" is the name in the hardware config on the driver station
////        in_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_in");
////        hold_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_hold");
////        out_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_out");
//        in_ColorSensor = new FastColorSensor(hardwareMap.get(I2cDeviceSynch.class, "Color_Sensor_in"));
//        hold_ColorSensor = new FastColorSensor(hardwareMap.get(I2cDeviceSynch.class, "Color_Sensor_hold"));
//        out_ColorSensor = new FastColorSensor(hardwareMap.get(I2cDeviceSynch.class, "Color_Sensor_out"));
//    }
//
//    @Override
//    public void loop() {
//        long start = System.nanoTime();
//        // These 3 calls now just pull from the Hub's pre-filled cache.
//        // Total time for all 3 should be < 1-2ms total.
//        in_ColorSensor.update();
//        hold_ColorSensor.update();
//        out_ColorSensor.update();
//
//        // PedroPathing Update
////        follower.update();
//
////        // Your Logic
////        if(sensorLeft.getProximity() > threshold) {
////            // do something
////        }
//
//        long end = System.nanoTime();
//
//        telemetry.addData("Read Time (ms)", (end - start) / 1_000_000.0);
//        telemetry.addData("Red", hold_ColorSensor.red());
//        telemetry.addData("Prox", hold_ColorSensor.getProximity());
//        telemetry.update();
//    }
}