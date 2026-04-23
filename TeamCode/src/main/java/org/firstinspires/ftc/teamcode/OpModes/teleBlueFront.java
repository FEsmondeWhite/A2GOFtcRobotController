package org.firstinspires.ftc.teamcode.OpModes;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(group = "TeleOp",name="TeleOp Blue front")
public class teleBlueFront extends ppTeleopBase {
    public teleBlueFront() {
        super(1, 1); // (AllianceColor, StartPosition)
//        private int AllianceColor = 1; // Blue is 1
//        private int AllianceColor = 2; // Red is 2
//        private int StartPosition = 1; // Front is 1
//        private int StartPosition = 2; // Back is 2
    }
}
