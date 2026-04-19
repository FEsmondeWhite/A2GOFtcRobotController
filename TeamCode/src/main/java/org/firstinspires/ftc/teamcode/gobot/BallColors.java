package org.firstinspires.ftc.teamcode.gobot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

public class BallColors {
    private HardwareMap hwMap;
    private Telemetry telemetry;

    private ColorSensor in_ColorSensor, hold_ColorSensor, out_ColorSensor;
    private DistanceSensor in_DistSensor, hold_DistSensor, out_DistSensor;

    private long lastUpdateTime = 0;
    private final long SENSOR_READ_INTERVAL = 100;

    private int in_r, in_g, in_b, hold_r, hold_g, hold_b, out_r, out_g, out_b;
    private Double in_dist, hold_dist, out_dist;
    private int in_ID, hold_ID, out_ID;
    private Character in_color = 'N', hold_color = 'N', out_color = 'N';

    private List<Character> colorList;
    private List<Character> patternList;
    private int patternIndex;
    private Telemetry.Item ballStatusItem;
    private String lastStatus = "";

    public BallColors(@NonNull HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init() {
        in_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_in");
        hold_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_hold");
        out_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_out");

        in_DistSensor = (DistanceSensor) in_ColorSensor;
        hold_DistSensor = (DistanceSensor) hold_ColorSensor;
        out_DistSensor = (DistanceSensor) out_ColorSensor;

        // Create a persistent line that starts as "N N N"
        ballStatusItem = telemetry.addData("Ball Colors", "In: N  Hold: N  Out: N");
        ballStatusItem.setRetained(true);
    }

    public void updateColors() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime < SENSOR_READ_INTERVAL) return;

        in_r = in_ColorSensor.red();
        in_g = in_ColorSensor.green();
        in_dist = in_DistSensor.getDistance(DistanceUnit.CM);

        hold_r = hold_ColorSensor.red();
        hold_g = hold_ColorSensor.green();
        hold_dist = hold_DistSensor.getDistance(DistanceUnit.CM);

        out_r = out_ColorSensor.red();
        out_g = out_ColorSensor.green();
        out_dist = out_DistSensor.getDistance(DistanceUnit.CM);

        identifyColors();
        lastUpdateTime = currentTime;
        telemetry();
    }

    public void identifyColors() {
        // In ball
        if (in_dist > 11.2) in_ID = 0;
        else if (1.0 * in_g / in_r >= 2.0) in_ID = 1;
        else in_ID = 2;

        // Hold ball
        if (hold_dist > 8) hold_ID = 0;
        else if (1.0 * hold_g / hold_r >= 1.8) hold_ID = 1;
        else hold_ID = 2;

        // Out ball
        if (out_dist > 5.5) out_ID = 0;
        else if (1.0 * out_g / out_r >= 1.8) out_ID = 1;
        else out_ID = 2;

        in_color = IDtoChar(in_ID);
        hold_color = IDtoChar(hold_ID);
        out_color = IDtoChar(out_ID);
        this.colorList = Arrays.asList(in_color, hold_color, out_color);
    }

    public Character IDtoChar(@NonNull int IDint) {
        if (IDint == 0) return 'N';
        if (IDint == 1) return 'G';
        if (IDint == 2) return 'P';
        if (IDint == 3) return 'Y';
        return 'E';
    }

    public List<Character> getColorList() {
        return colorList;
    }

    public void setPatternList(List<Character> patternList) {
        this.patternList = patternList;
    }

    public List<Character> getPatternList() {
        return patternList;
    }
    // Inside BallColors.java
    public void detailedTelemetry() {
        telemetry.addData("In", "G/R: %.2f Dist: %.2f", (1.0*in_g/in_r), in_dist);
        telemetry.addData("Hold", "G/R: %.2f Dist: %.2f", (1.0*hold_g/hold_r), hold_dist);
        telemetry.addData("Out", "G/R: %.2f Dist: %.2f", (1.0*out_g/out_r), out_dist);
    }
    public void telemetry() {
        // Only build the string if we actually have to
        String currentStatus = String.format("In: %c  Hold: %c  Out: %c", in_color, hold_color, out_color);

        if (!currentStatus.equals(lastStatus)) {
            ballStatusItem.setValue(currentStatus); // Efficiently update the existing line
            lastStatus = currentStatus;
        }
    }

    public boolean anyBallReadyForLaunch() {
        return (this.out_color != 'N');
    }

    public boolean anyBallAvailable() {
        for (char c : this.colorList) {
            if (c != 'N') {
                return true; // Found a character that is not 'N'
            }
        }
        return false; // All characters are 'N'
    }

    public boolean targetColorAvailable() {
        return (colorList.contains(this.targetPatternColor()));
    }

    public boolean targetMatchForLaunch() {
        return (this.out_color == this.targetPatternColor());
    }

    public Character targetPatternColor() {
        return this.patternList.get(this.patternIndex);
    }

    public boolean validPatternIndex() {
        if ((this.patternIndex >=0) && (this.patternIndex <=2)) {
            return true;
        } else {
            return false;
        }
    }
}
