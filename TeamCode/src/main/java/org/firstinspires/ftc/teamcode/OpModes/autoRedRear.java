package org.firstinspires.ftc.teamcode.OpModes;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous(group = "Auto",name="Auto Red rear", preselectTeleOp="TeleOp Red rear")
public class autoRedRear extends ppAutoBase {
    public autoRedRear() {
        super(2, 2); // (AllianceColor, StartPosition)
//        private int AllianceColor = 1; // Blue is 1
//        private int AllianceColor = 2; // Red is 2
//        private int StartPosition = 1; // Front is 1
//        private int StartPosition = 2; // Back is 2
    }
}
