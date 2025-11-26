package org.firstinspires.ftc.teamcode.gobot;

public class AprilTags {
    //    Blue Goal: 20
    //    Red Goal: 24
    int GoalID;
    int motifGPP = 21;
    int motifPGP = 22;
    int motifPPG = 23;
    //    Motif GPP: 21
    //    Motif PGP: 22
    //    Motif PPG: 23

    public AprilTags(int AllianceColor) {
        if (AllianceColor == 1) {
            GoalID = 20;
            //    Blue Goal: 20
        } else {
            GoalID = 24;
            //    Red Goal: 24
        }
    }
}
