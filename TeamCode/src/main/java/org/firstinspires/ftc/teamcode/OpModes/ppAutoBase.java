package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gobot.Intake;
import org.firstinspires.ftc.teamcode.gobot.Launcher;
import org.firstinspires.ftc.teamcode.gobot.Lifter;
import org.firstinspires.ftc.teamcode.gobot.PoseStorage;
import org.firstinspires.ftc.teamcode.gobot.Sorter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.gobot.VisionDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import android.util.Size;

// Note that we want to have auto pre-load the tele routines. This is done in the different instances.
//https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/auto_load_opmode/auto-load-opmode.html
//@Autonomous()
abstract public class ppAutoBase extends OpMode {

    // Define the VisionDetection utility object
    private VisionDetection vision;
    private static final boolean MANUAL_CAMERA = true;

    // Manual camera settings (adjust based on testing)
    private static final Size CAMERA_RESOLUTION = new Size(640, 480);
    public static int EXPOSURE_MS = 30; // Optimal for minimizing motion blur
    public static int GAIN_VAL = 255;   // High gain to compensate for low exposure
    int patternID;
    List<Character> colorPattern;
    private int currentBall;

    private Follower follower;

    //Timers and flags
    private ElapsedTime taskTimer;
    private boolean taskFlag=false;

    private final ArrayList<PathChain> pathA = new ArrayList<>();

    enum TaskState {
        TASK_IDLE,      //Prior to start
        TASK_LIFTING,   //Lift/launch
        TASK_SORTING,     //Find the new ball
        TASK_SCANNING, // Use camera to scan
        TASK_DONE
    }
    TaskState taskState = TaskState.TASK_IDLE;

    enum State {
        IDLE,      //Prior to start
        TRAJ_0,   //Move to scan april tag pose
        SCAN,     //Scan april tag
        TRAJ_1,   //Move to shoot pose
        SHOOT_A,   //Shoot 3 balls
        TRAJ_2,   //Move in front of first stack
        TRAJ_3,   //Collect ball 1
        TRAJ_4,   //Collect ball 2
        TRAJ_5,   //Collect ball 3
        TRAJ_6,   //Move to shoot pose
        SHOOT_B,   //Shoot 3 balls
        TRAJ_7,   //Move to end pose
        STOP    // Wait for TeleOp
    }
    State currentState = State.IDLE;

    private TelemetryManager telemetryM;

    private int AllianceColor;
//    private int AllianceColor = 1; // Blue is 1
//    private int AllianceColor = 2; // Red is 2
    private int StartPosition;
//    private int StartPosition = 1; // Front is 1
//    private int StartPosition = 1; // Back is 2
    private int taskCount;

    public static Pose startingPose;  //See ExampleAuto to understand how to use this

    private Pose shootPose;
    double shootSpeedRPS;
    double rearSpeedRPS = 60;
    double frontSpeedRPS = 55;
    private Pose frontShootPose;
    private Pose rearShootPose;
    private Pose endPose;
    private Pose scanAprilTagPose;
    private Pose intakeLineup1Pose;
    private Pose intake1Ball1GPose;
    private Pose intake1Ball2PPose;
    private Pose intake1Ball3PPose;
    private Pose intakeLineup2Pose;
    private Pose intake2Ball1PPose;
    private Pose intake2Ball2GPose;
    private Pose intake2Ball3PPose;
    private Pose intakeLineup3Pose;
    private Pose intake3Ball1PPose;
    private Pose intake3Ball2PPose;
    private Pose intake3Ball3GPose;

    private Pose stack1StartPose;
    private Pose stack1Ball1Pose;
    private Pose stack1Ball2Pose;
    private Pose stack1Ball3Pose;


    public void buildPaths() {
        //Set initial power
        follower.setMaxPower(1.0);

        //[0] Move to scan pose
        // TRAJ_0,   //Move to scan april tag pose
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(startingPose, scanAprilTagPose))
                        .setLinearHeadingInterpolation(startingPose.getHeading(), scanAprilTagPose.getHeading())
                        .build()
        );
//        SCAN,     //Scan april tag, no moving

        //[1] shootPose
        // TRAJ_1,   //Move to shoot pose
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(scanAprilTagPose, shootPose))
                        .setLinearHeadingInterpolation(scanAprilTagPose.getHeading(), shootPose.getHeading())
                        .build()
        );

//        SHOOT_A,   //Shoot 3 balls, no motion

        //[2] Move to first stack for intake
        // TRAJ_2,   //Move in front of first stack
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, stack1StartPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), stack1StartPose.getHeading())
                        .build()
        );


        //[3] Collect first artifact
        // TRAJ_3,   //Collect ball 1
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(stack1StartPose, stack1Ball1Pose))
                        .setLinearHeadingInterpolation(stack1StartPose.getHeading(), stack1Ball1Pose.getHeading())
                        .build()
        );

        //[4] Collect second artifact
        // TRAJ_4,   //Collect ball 2
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(stack1Ball1Pose, stack1Ball2Pose))
                        .setLinearHeadingInterpolation(stack1Ball1Pose.getHeading(), stack1Ball2Pose.getHeading())
                        .build()
        );

        //[5] Collect third artifact
        // TRAJ_5,   //Collect ball 3
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(stack1Ball2Pose, stack1Ball3Pose))
                        .setLinearHeadingInterpolation(stack1Ball2Pose.getHeading(), stack1Ball3Pose.getHeading())
                        .build()
        );

        //[6] shootPose
        // TRAJ_6,   //Move to shoot pose
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(stack1Ball3Pose, shootPose))
                        .setLinearHeadingInterpolation(stack1Ball3Pose.getHeading(), shootPose.getHeading())
                        .build()
        );

//        SHOOT_B,   //Shoot 3 balls

        //[7] endPose
        // TRAJ_7,   //Move to end pose
        pathA.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, endPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                        .build()
        );
    }

    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    private Launcher launcher;

    //    private int AllianceColor = 1; // Blue is 1
    //    private int AllianceColor = 2; // Red is 2
    //    private int StartPosition = 1; // Front is 1
    //    private int StartPosition = 1; // Back is 2

    // Get pose and heading based on alliance color and start position (front/back)
    public ppAutoBase(int AllianceColor, int StartPosition) {
        this.AllianceColor = AllianceColor;
        this.StartPosition = StartPosition;
        if (AllianceColor == 1) {
            if (StartPosition == 1) {
                startingPose = new Pose(57.5, 9, 1.57079); // Math.toRadians(90)
            } else if (StartPosition == 2) {
                startingPose = new Pose(33.5, 134, 1.57079); // 270 degrees
            }
        } else if (AllianceColor == 2) {
            if (StartPosition == 1) {
                startingPose = new Pose(144 - 57.5, 9, 1.57079); // Math.toRadians(90)
            } else if (StartPosition == 2) {
                startingPose = new Pose(144 - 33.5, 134, 1.57079); // 270 degrees
            }
        }
        currentBall = 0; // Set the pattern position to ball 0

        // In auto, we should always start in the new position.
//        // If there is a stored pose (starting from PP auto) then
//        // Get the pose from storage
//        if(!( PoseStorage.currentPose==null) ) {
//            // This will load the pose if we have already run the robot.
//            startingPose = PoseStorage.currentPose;
//        }

        // Define the field poses for shooting and parking based on alliance color
        if (AllianceColor == 1) {
            frontShootPose = new Pose(59,14, (double) Math.toRadians(115));
            rearShootPose = new Pose(58,85, (double) Math.toRadians(134));
            if (StartPosition == 1) {
                shootPose = frontShootPose;
                shootSpeedRPS = frontSpeedRPS;
                endPose = new Pose(48,25, 1.57079); // Math.toRadians(90));
            } else if (StartPosition == 2) {
                shootPose = rearShootPose;
                shootSpeedRPS = rearSpeedRPS;
                endPose = new Pose(48,128, 4.71239); // 270 degrees
            }
        } else if (AllianceColor == 2) {
            frontShootPose = new Pose(86,14, (double) Math.toRadians(66)); // Math.toRadians(66.5));
            rearShootPose = new Pose(88,85, (double) Math.toRadians(49)); // Math.toRadians(180-133)); 47 deg
            if (StartPosition == 1) {
                shootPose = frontShootPose;
                shootSpeedRPS = frontSpeedRPS;
                endPose = new Pose(96,25, 1.57079); // Math.toRadians(90));
            } else if (StartPosition == 2) {
                shootPose = rearShootPose;
                shootSpeedRPS = rearSpeedRPS;
                endPose = new Pose(96,128, 4.71239); // 270 degrees
            }
        }

        // Ball Intake Locations
        if (AllianceColor == 1) {
            // Blue intake locations
            if (StartPosition == 1) {
                scanAprilTagPose = new Pose(58.5, 25, (double) Math.toRadians(83));
            } else if (StartPosition == 2) {
                scanAprilTagPose = new Pose(55, 110, (double) Math.toRadians(67));
            }
            intakeLineup1Pose = new Pose(42, 36,(double) Math.toRadians(180));
            intake1Ball1GPose = new Pose(36, 36,(double) Math.toRadians(180));
            intake1Ball2PPose = new Pose(31, 36,(double) Math.toRadians(180));
            intake1Ball3PPose = new Pose(26, 36,(double) Math.toRadians(180));
            intakeLineup2Pose = new Pose(42, 60,(double) Math.toRadians(180));
            intake2Ball1PPose = new Pose(36, 60,(double) Math.toRadians(180));
            intake2Ball2GPose = new Pose(31, 60,(double) Math.toRadians(180));
            intake2Ball3PPose = new Pose(26, 60,(double) Math.toRadians(180));
            intakeLineup3Pose = new Pose(42, 84,(double) Math.toRadians(180));
            intake3Ball1PPose = new Pose(36, 84,(double) Math.toRadians(180));
            intake3Ball2PPose = new Pose(31, 84,(double) Math.toRadians(180));
            intake3Ball3GPose = new Pose(26, 84,(double) Math.toRadians(180));

            if (StartPosition == 1) {
                stack1StartPose = intakeLineup1Pose;
                stack1Ball1Pose = intake1Ball1GPose;
                stack1Ball2Pose = intake1Ball2PPose;
                stack1Ball3Pose = intake1Ball3PPose;
            } else if (StartPosition == 2) {
                stack1StartPose = intakeLineup3Pose;
                stack1Ball1Pose = intake3Ball1PPose;
                stack1Ball2Pose = intake3Ball2PPose;
                stack1Ball3Pose = intake3Ball3GPose;
            }
        } else if (AllianceColor == 2) {
            // Red intake locations
            if (StartPosition == 1) {
                scanAprilTagPose = new Pose(85.5, 25, (double) Math.toRadians(97));
            } else if (StartPosition == 2) {
                scanAprilTagPose = new Pose(90, 110, (double) Math.toRadians(113));
            }
            intakeLineup1Pose = new Pose(102, 35,(double) Math.toRadians(0));
            intake1Ball1GPose = new Pose(108, 35,(double) Math.toRadians(0));
            intake1Ball2PPose = new Pose(113, 35,(double) Math.toRadians(0));
            intake1Ball3PPose = new Pose(118, 35,(double) Math.toRadians(0));
            intakeLineup2Pose = new Pose(102, 59,(double) Math.toRadians(0));
            intake2Ball1PPose = new Pose(108, 59,(double) Math.toRadians(0));
            intake2Ball2GPose = new Pose(113, 59,(double) Math.toRadians(0));
            intake2Ball3PPose = new Pose(118, 59,(double) Math.toRadians(0));
            intakeLineup3Pose = new Pose(102, 83,(double) Math.toRadians(0));
            intake3Ball1PPose = new Pose(108, 83,(double) Math.toRadians(0));
            intake3Ball2PPose = new Pose(113, 83,(double) Math.toRadians(0));
            intake3Ball3GPose = new Pose(118, 83,(double) Math.toRadians(0));

            if (StartPosition == 1) {
                stack1StartPose = intakeLineup1Pose;
                stack1Ball1Pose = intake1Ball1GPose;
                stack1Ball2Pose = intake1Ball2PPose;
                stack1Ball3Pose = intake1Ball3PPose;
            } else if (StartPosition == 2) {
                stack1StartPose = intakeLineup3Pose;
                stack1Ball1Pose = intake3Ball1PPose;
                stack1Ball2Pose = intake3Ball2PPose;
                stack1Ball3Pose = intake3Ball3GPose;
            }
        }
    }

    @Override
    public void init() {
        intake = new Intake();
        intake.init(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        sorter.init();
        lifter = new Lifter();
        lifter.init(hardwareMap);
        launcher = new Launcher();
        launcher.init(hardwareMap);
        taskCount = 0;

        // Initialize the Vision Detection utility
        vision = new VisionDetection(
                hardwareMap,
                AllianceColor,
                MANUAL_CAMERA,
                CAMERA_RESOLUTION,
                EXPOSURE_MS,
                GAIN_VAL
        );

        taskTimer=new ElapsedTime();
        //Drivetrain (PP)
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startingPose);

        // telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }


    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work

        sorter.update();
        lifter.update();

        updatePath();

        //Call this once per loop
        follower.update();

        this.sorter.balls.telemetry();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
//        telemetryM.update();

        // Read current pose and save to storage
//        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = follower.getPose();
    }
        // Intake
//            intake.setIntake(2); // intake
//            intake.setIntake(-2); // push out
//            intake.setIntake(0); // turn off

        // Sorter
//        if (!lifter.isBusy()) {
//                sorter.start(1); // Sort the ball
//        }

        // Lifter
//        if (!sorter.isBusy()) {
//                lifter.start(); // Lift/launch the ball
//        }

        // Flywheel
//            launcher.setNominalRPS(shootSpeedRPS);
//            launcher.setNominalRPS(0);

        // static public flywheel PIDF allows it to be changed in panels
//            launcher.enableMotor(); // turn on flywheel
//            telemetry.addData("Flywheel setpoint: ", launcher.getNominalRPS());
//            telemetry.addData("Flywheel actual: ", launcher.getActualRPS());
//        telemetryM.addData("Flywheel setpoint", launcher.getNominalRPS());
//        telemetryM.addData("Flywheel actual", launcher.getActualRPS());
//
//            launcher.disableMotor(); // turn off flywheel

    public void updatePath() {
        switch (currentState) {
            //Prior to start
            case IDLE:
                currentState = State.TRAJ_0;
                break;

            //[0] Move to scan pose
            case TRAJ_0:
                if (!follower.isBusy()) {

                    // Start the flywheel
                    launcher.enableMotor(); // turn on flywheel
                    launcher.setNominalRPS(shootSpeedRPS);

                    follower.followPath(pathA.get(0), true);
                    currentState = State.SCAN;
                    taskFlag = false;
                    taskState = TaskState.TASK_IDLE;
                }
                break;

            // Scan the april tag
            case SCAN:
                if (!follower.isBusy()) {
                    if (taskState == TaskState.TASK_IDLE) {
                        taskTimer.reset(); // Reset & start the timer
                        taskState = TaskState.TASK_SCANNING;
                    }
                    if (!taskFlag) {
                        // Add code to scan the april tag!
                        patternID = vision.getDetectedPatternID();
                        if (patternID != 0) {
                            // Feedback to Driver Hub for debugging
                            telemetry.addData("Pattern Identified: ", patternID);
                            colorPattern = vision.getArtifactColorPattern();
                            this.sorter.balls.setPatternList(colorPattern);
                            taskFlag = true; // Done scanning, we have the pattern
                            // Then move to the next position
                            currentState = State.TRAJ_1;
                        }
                    } else if (taskTimer.milliseconds() > 1000) {
                        // We didn't find the pattern within one second
                        patternID = 22; // We will guess the pattern and move on
                        colorPattern = Arrays.asList('P', 'G', 'P'); // Assuming PGP, P starts 2/3 patterns. We will pre-load PGP.
                        this.sorter.balls.setPatternList(colorPattern);
                    }
                }
                break;

            //[1] Move to shoot pose
            case TRAJ_1:
                if (!follower.isBusy()) {
                    // Move to the shoot position
                    follower.followPath(pathA.get(1), true);
                    currentState = State.SHOOT_A;
                    // Configure for the shooting state machine
//                    taskFlag = false;
//                    taskCount = 3;
                    taskState = TaskState.TASK_IDLE;
                }
                break;

//            launchState
//                IDLE,      //Prior to start
//                LIFTING,   //Lift/launch
//                SORTING,     //Find the new ball
//                DONE

            //Shoot 3 Balls
            case SHOOT_A:
                // We need to add code to look for the pattern colored balls before launching.
                if (!follower.isBusy()) {
                    //SHOOT SEQUENCE HERE
                    this.sorter.balls.updateColors();
                    // We will look at the current task state.
                    // If we're not doing something, then check if we have the correct ball, or can get a ball.
                    // We will launch if have the right ball, or a ball (if the correct ball isn't available).
                    // We will sort if there are balls, but we don't have the correct one (or any ball).
                    // If there are no balls loaded, then we're done launching.
                    switch (taskState) {
                        case TASK_IDLE:
                            // We have a list of the actual ball colors in the sorter:
                            //   this.sorter.balls.getColorList()
                            // We have a list of the desired pattern:
                            //   this.colorPattern
                            // And we know the ball we're currently wanting to launch:
                            //   this.currentBall

                            // We will shoot up to the specified number (taskCount) of balls.
                            // First, we need to sort to line up the correct ball, if available.
                            // Then we will shoot the ball
                            //      we need to lift the ball
                            // Then we will move on
                            if (this.sorter.balls.targetMatchForLaunch()) {
                                // if the correct ball is already lined up, then launch.
                                if (!sorter.isBusy()) {
                                    lifter.start(); // Lift/launch the ball
                                    taskState = TaskState.TASK_LIFTING;
                                }

                            } else if (this.sorter.balls.targetColorAvailable()) {
                                // if the correct ball is available, but not lined up, then sort.
                                if (!lifter.isBusy()) {
                                    sorter.start(1); // Lift/launch the ball
                                    taskState = TaskState.TASK_SORTING;
                                }

                            } else if (this.sorter.balls.anyBallReadyForLaunch()) {
                                // if the correct ball is not available, but we have a ball, then launch.
                                if (!sorter.isBusy()) {
                                    lifter.start(); // Lift/launch the ball
                                    taskState = TaskState.TASK_LIFTING;
                                }

                            } else if (this.sorter.balls.anyBallAvailable()) {
                                // if the correct ball is not available, but we do have a ball somewhere, then sort.
                                if (!lifter.isBusy()) {
                                    sorter.start(1); // Lift/launch the ball
                                    taskState = TaskState.TASK_SORTING;
                                }

                            } else {
                                // if no ball is available, then we're done. Move to the next state.
                                taskState = TaskState.TASK_IDLE;
                                launcher.disableMotor(); // turn off flywheel
                                currentState = State.TRAJ_2;
                            }
                            break;
                        case TASK_LIFTING:
                            if (!lifter.isBusy()) { // This will be true when it's done lifting
                                taskState = TaskState.TASK_IDLE;
                            }
                            break;
                        case TASK_SORTING:
                            if (!sorter.isBusy()) { // This will be true when it's done sorting
                                taskState = TaskState.TASK_IDLE;
                            }
                        break;
                    }
                }
                break;


            //[2] Move in front of first stack
            case TRAJ_2:
                if (!follower.isBusy()) {
                    intake.setIntake(2); // intake
                    follower.followPath(pathA.get(2), true);
                    taskState = TaskState.TASK_IDLE;
                    currentState = State.TRAJ_3;
                }
                break;


            //[3] Collect ball 1
            case TRAJ_3:
                if (!follower.isBusy()) {
                    switch (taskState) {
                        case TASK_IDLE:
                            // Rotate the ball sorter
                            if (!lifter.isBusy()) {
                                sorter.start(1); // turn the ball sorter
                                intake.setIntake(-2); // reverse the intake
                                taskState = TaskState.TASK_SORTING;
                            }
                            break;
                        case TASK_SORTING:
                            if (!sorter.isBusy()) {
                                intake.setIntake(2); // turn the intake back on
                                follower.setMaxPower(0.2); //SLOW DOWN!!!
                                follower.followPath(pathA.get(3), true);
                                taskState = TaskState.TASK_IDLE;
                                currentState = State.TRAJ_4;
                            }
                            break;
                    }
                }
                break;

            //[4] Collect ball 2
            case TRAJ_4:
                if (!follower.isBusy()) {
                    switch (taskState) {
                        case TASK_IDLE:
                            // Rotate the ball sorter
                            if (!lifter.isBusy()) {
                                sorter.start(1); // Lift/launch the ball
                                intake.setIntake(-2); // reverse the intake
                                taskState = TaskState.TASK_SORTING;
                            }
                            break;
                        case TASK_SORTING:
                            if (!sorter.isBusy()) {
                                intake.setIntake(2); // turn the intake back on
                                follower.followPath(pathA.get(4), true);
                                taskState = TaskState.TASK_IDLE;
                                currentState = State.TRAJ_5;
                            }
                            break;
                    }
                }
                break;

            //[5] Collect ball 3
            case TRAJ_5:
                if (!follower.isBusy()) {
                    switch (taskState) {
                        case TASK_IDLE:
                            // Rotate the ball sorter
                            if (!lifter.isBusy()) {
                                sorter.start(1); // Lift/launch the ball
                                intake.setIntake(-2); // reverse the intake
                                taskState = TaskState.TASK_SORTING;
                            }
                            break;
                        case TASK_SORTING:
                            if (!sorter.isBusy()) {
                                intake.setIntake(2); // turn the intake back on
                                taskState = TaskState.TASK_IDLE;
                                taskTimer.reset(); // Start the timer. Wait 1.5 seconds to pull in ball #3.
                                follower.followPath(pathA.get(5), true);
                                currentState = State.TRAJ_6;
                            }
                            break;
                    }
                }
                break;

            //[6] Move to shoot pose
            case TRAJ_6:
                if (!follower.isBusy() && taskTimer.time() > 1.5) {
                    intake.setIntake(0); // turn the intake off
                    follower.followPath(pathA.get(6), true);
                    follower.setMaxPower(1.0); //BACK TO FULL SPEED
                    currentState = State.SHOOT_B;
                    taskFlag = false;
                    launcher.enableMotor(); // turn on flywheel
                    taskCount = 3;
                    taskState = TaskState.TASK_IDLE;
                }
                break;

            //Shoot 3 Balls
            case SHOOT_B:
                if (!follower.isBusy()) {
                    if (!taskFlag) {
                        //SHOOT SEQUENCE HERE
                        if (taskCount>0) {
                            switch (taskState) {
                                case TASK_IDLE:
                                    if (!sorter.isBusy()) {
                                        lifter.start(); // Lift/launch the ball
                                        taskState = TaskState.TASK_LIFTING;
                                    }
                                    break;
                                case TASK_LIFTING:
                                    if (!lifter.isBusy()) {
                                        sorter.start(1); // Lift/launch the ball
                                        taskState = TaskState.TASK_SORTING;
                                    }
                                    break;
                                case TASK_SORTING:
                                    if (!sorter.isBusy()) {
                                        taskState = TaskState.TASK_IDLE;
                                        taskCount=taskCount-1;
                                        if (taskCount==0) {
                                            launcher.disableMotor(); // turn off flywheel
                                            taskFlag = true;
                                        }
                                    }
                                    break;
                            }

                        } else {
                            taskFlag = true;
                        }

                    } else if (taskFlag) { //&& taskTimer.time() > 2.
                        taskFlag = false;
                        currentState = State.TRAJ_7;
                        follower.followPath(pathA.get(7), true);
                    }
                }
                break;

            //[7] Move to STOP pose
            case TRAJ_7:
                if (!follower.isBusy()) {
                    // follower.followPath(pathA.get(7), true);
                    currentState = State.STOP;
                }
                break;

            case STOP:
                //Nothing
                break;
        }
    }
}
