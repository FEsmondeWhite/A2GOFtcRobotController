package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gobot.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.gobot.Intake;

@Disabled
@Autonomous(group = "TEST",name="Test: Blue front auto PedroPath")
public class oldTestBlueFrontPPIntake extends OpMode {
    private Follower follower;

    Intake intake = new Intake();

    //Timers and flags
    private ElapsedTime taskTimer;
    private boolean taskFlag=false;

    //Define points along paths
    private final Pose startPose = new Pose(54.000, 43.000, Math.toRadians(180));
    private final Pose grabPose = new Pose(30.000, 43.000, Math.toRadians(180));
    private final Pose endPose = new Pose(35.000, 43.000, Math.toRadians(180));

//    private final Pose shootPose = new Pose(63, 84.6, Math.toRadians(130));
//    private final Pose stackStartPose = new Pose(63, 80, Math.toRadians(180));
//    private final Pose stackEndPose = new Pose(38, 80, Math.toRadians(180));
//    private final Pose parkPose = new Pose(63, 84.6, Math.toRadians(180));

    private final ArrayList<PathChain> pathA = new ArrayList<>();

    enum State {
        IDLE,      //Prior to start
        TRAJ_0,   //Move to shoot pose
        TRAJ_1,   //Move to shoot pose
//        SHOOT_A,   //Shoot 3 balls
//        TRAJ_2,   //Move in front of first stack
//        TRAJ_3,   //Collect 3 balls
//        TRAJ_4,   //Move to shoot pose
//        SHOOT_B,   //Shoot 3 balls
//        TRAJ_5,   //Move to park pose
        PARK
    }
    State currentState = State.IDLE;

    public void buildPaths() {
        //Set initial power
        follower.setMaxPower(0.3);

        //[0] Move to shoot pose
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(startPose, grabPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), grabPose.getHeading())
                        .build()
        );
        //[1] Move to shoot pose
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(grabPose, endPose))
                        .setLinearHeadingInterpolation(grabPose.getHeading(), endPose.getHeading())
                        .build()
        );
//
//        //[2] Move in front of first stack
//        pathA.add(
//                follower.pathBuilder()
//                        .addPath(new BezierLine(shootPose, stackStartPose))
//                        .setLinearHeadingInterpolation(shootPose.getHeading(), stackStartPose.getHeading())
//                        .build()
//        );
//
//        //[3] Collect 3 balls
//        pathA.add(
//                follower.pathBuilder()
//                        .addPath(new BezierLine(stackStartPose, stackEndPose))
//                        .setLinearHeadingInterpolation(stackStartPose.getHeading(), stackEndPose.getHeading())
//                        .build()
//        );
//
//        //[4] Move to shoot pose
//        pathA.add(
//                follower.pathBuilder()
//                        .addPath(new BezierLine(stackEndPose, shootPose))
//                        .setLinearHeadingInterpolation(stackEndPose.getHeading(), shootPose.getHeading())
//                        .build()
//        );
//
//        //[5] Move to park pose
//        pathA.add(
//                follower.pathBuilder()
//                        .addPath(new BezierLine(shootPose, parkPose))
//                        .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
//                        .build()
//        );
    }

    @Override
    public void init() {
        intake.init(hardwareMap);
        taskTimer=new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        updatePath();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    public void stop() {
        // Read current pose and save to storage
        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;
    }

    public void updatePath() {
        switch (currentState) {
            //Prior to start
            case IDLE:
                currentState = State.TRAJ_0;
                break;

            //[0] Move to scan pose
            case TRAJ_0:
                if (!follower.isBusy()) {
                    intake.setIntake(2);
                    follower.followPath(pathA.get(0), true);
                    currentState = State.TRAJ_1;
                }
                break;

            //[1] Load ball and move back
            case TRAJ_1:
                if (!follower.isBusy()) {
                    if (!taskFlag) {
                        //Intake SEQUENCE HERE
                        taskFlag = true;
                        taskTimer.reset();
                    } else if (taskFlag && taskTimer.time() > 1) {
                        taskFlag = false;
                        intake.setIntake(0);
                        follower.followPath(pathA.get(1), true);
                        currentState = State.PARK;
                    }
                }
                break;
//
//            //Shoot 3 Balls
//            case SHOOT_A:
//                if (!follower.isBusy()) {
//                    if (!taskFlag) {
//                        //SHOOT SEQUENCE HERE
//
//                        taskFlag = true;
//                        taskTimer.reset();
//                    } else if (taskFlag && taskTimer.time() > 2) {
//                        taskFlag = false;
//                        currentState = State.TRAJ_2;
//                    }
//                }
//                break;
//
//            //[2] Move in front of first stack
//            case TRAJ_2:
//                if (!follower.isBusy()) {
//                    follower.followPath(pathA.get(2), true);
//                    currentState = State.TRAJ_3;
//                }
//                break;
//
//
//            //[3] Collect 3 balls
//            case TRAJ_3:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(0.2); //SLOW DOWN!!!
//                    follower.followPath(pathA.get(3), true);
//                    currentState = State.TRAJ_4;
//                }
//                break;
//
//            //[4] Move to shoot pose
//            case TRAJ_4:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1.0); //BACK TO FULL SPEED
//                    follower.followPath(pathA.get(4), true);
//                    currentState = State.SHOOT_B;
//                }
//                break;
//
//            //Shoot 3 Balls
//            case SHOOT_B:
//                if (!follower.isBusy()) {
//                    if (!taskFlag) {
//                        //SHOOT SEQUENCE HERE
//
//                        taskFlag = true;
//                        taskTimer.reset();
//                    } else if (taskFlag && taskTimer.time() > 2) {
//                        taskFlag = false;
//                        currentState = State.TRAJ_5;
//                    }
//                }
//                break;
//
//            //[5] Move to park pose
//            case TRAJ_5:
//                if (!follower.isBusy()) {
//                    follower.followPath(pathA.get(5), true);
//                    currentState = State.PARK;
//                }
//                break;

            case PARK:
                //Nothing
                break;
        }
    }
}
