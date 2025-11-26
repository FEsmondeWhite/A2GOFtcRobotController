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
    // Determine which balls are present, and what color they are

    // If the distance is above the threshold, there is no ball present (ID = 0)
    // In color sensor dist threshold is 8
    // Hold color sensor dist threshold is 8
    // Out color sensor dist threshold is 6

    // G/R above 2 means green (ID = 1). Otherwise purple (ID = 2)

    // No ball shows character 'N'. Green shows 'G'. Purple shows 'P'.

    private HardwareMap hwMap;
    private Telemetry telemetry;

    private ColorSensor in_ColorSensor;
    private int in_r, in_g, in_b;
    private Double in_dist;
    private int in_ID;
    private Character in_color = 'N';

    private ColorSensor hold_ColorSensor;
    private int hold_r, hold_g, hold_b;
    private Double hold_dist;
    private int hold_ID;
    private Character hold_color = 'N';

    private ColorSensor out_ColorSensor;
    private int out_r, out_g, out_b;
    private Double out_dist;
    private int out_ID;
    private Character out_color = 'N';

    private List<Character> colorList; // list of currently held balls

    private List<Character> patternList; // target pattern colors

    public int getPatternIndex() {
        return patternIndex;
    }

    public void setPatternIndex(int patternIndex) {
        this.patternIndex = patternIndex;
    }

    private int patternIndex;

    public BallColors(@NonNull HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init() {
        in_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_in");
        hold_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_hold");
        out_ColorSensor = hwMap.get(RevColorSensorV3.class, "Color_Sensor_out");
    }

    public void updateColors() {
        in_r=in_ColorSensor.red();
        in_g=in_ColorSensor.green();
        in_b=in_ColorSensor.blue();
        in_dist = ((DistanceSensor) in_ColorSensor).getDistance(DistanceUnit.CM); // Get distance in centimeters
        // You can also use DistanceUnit.INCH or DistanceUnit.MM

        hold_r=hold_ColorSensor.red();
        hold_g=hold_ColorSensor.green();
        hold_b=hold_ColorSensor.blue();
        hold_dist = ((DistanceSensor) hold_ColorSensor).getDistance(DistanceUnit.CM); // Get distance in centimeters

        out_r=out_ColorSensor.red();
        out_g=out_ColorSensor.green();
        out_b=out_ColorSensor.blue();
        out_dist = ((DistanceSensor) out_ColorSensor).getDistance(DistanceUnit.CM); // Get distance in centimeters

        identifyColors();
        return;
    }

    public void identifyColors() {
        // In ball
        if (in_dist>12) {
            // no ball
            in_ID = 0;
        } else if (1.0*in_g / in_r >= 2.0) {
            // ball is green
            in_ID = 1;
        } else {
            //ball is purple
            in_ID = 2;
        }

        // Hold ball
        if (hold_dist>8) {
            // no ball
            hold_ID = 0;
        } else if (1.0*hold_g / hold_r >= 1.8) {
            // ball is green
            hold_ID = 1;
        } else {
            //ball is purple
            hold_ID = 2;
        }

        // Out ball
        if (out_dist>6) {
            // no ball
            out_ID = 0;
        } else if (1.0*out_g / out_r >= 1.8) {
            // ball is green
            out_ID = 1;
        } else {
            //ball is purple
            out_ID = 2;
        }
        in_color = IDtoChar(in_ID);
        hold_color = IDtoChar(hold_ID);
        out_color = IDtoChar(out_ID);
        this.colorList = Arrays.asList(in_color, hold_color, out_color);
        return;
    }

    // IDtoChar converts the integer ball color code into a character.
    //    0: No ball 'N', 1: Green ball 'G', 2: Purple ball 'P'
    //    // -> this is not returned anymore. Yellow should show as purple instead. 3: Yellow ball",
    public Character IDtoChar(@NonNull int IDint)  {
        Character IDchar;

        if (IDint == 0) { // No ball
            IDchar = 'N';
        } else if (IDint == 1) { // Green ball
            IDchar = 'G';
        } else if (IDint == 2) { // Purple ball
            IDchar = 'P';
        } else if (IDint == 3) { // Yellow ball
            IDchar = 'Y';
        } else { // Error!
            IDchar = 'E';
        }
        return IDchar;
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


    public void detailedTelemetry() {
        // Initialize the string
        String statusMessage = "";

        // Add characters/strings using concatenation
        statusMessage += " G/R: ";
        statusMessage += 10*in_g / in_r;
        statusMessage += "\t Dist: ";
        statusMessage += in_dist;

        telemetry.addData("In: ", statusMessage);
        statusMessage = "";

        // Add characters/strings using concatenation
        statusMessage += " G/R: ";
        statusMessage += 10*hold_g / hold_r;
        statusMessage += "\t Dist: ";
        statusMessage += hold_dist;

        telemetry.addData("Hold: ", statusMessage);
        statusMessage = "";

        // Add characters/strings using concatenation
        statusMessage += " G/R: ";
        statusMessage += 10*out_g / out_r;
        statusMessage += "\t Dist: ";
        statusMessage += out_dist;

        telemetry.addData("Out: ", statusMessage);
//        telemetry.addLine("This is a secondary line of text.");
//        telemetry.update();
    }

    public void telemetry() {
        // Initialize the string
        String statusMessage = "";

        // Add characters/strings using concatenation
        statusMessage += " In: ";
        statusMessage += this.in_color;
        statusMessage += " Hold: ";
        statusMessage += this.hold_color;
        statusMessage += " Out: ";
        statusMessage += this.out_color;

        telemetry.addData("Ball Colors. ", statusMessage);
//        telemetry.addLine("This is a secondary line of text.");
//        telemetry.update();
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
