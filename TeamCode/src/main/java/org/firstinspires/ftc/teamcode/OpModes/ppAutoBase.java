package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gobot.EnPointe;
import org.firstinspires.ftc.teamcode.gobot.Intake;
import org.firstinspires.ftc.teamcode.gobot.Launcher;
import org.firstinspires.ftc.teamcode.gobot.Lifter;
import org.firstinspires.ftc.teamcode.gobot.PoseStorage;
import org.firstinspires.ftc.teamcode.gobot.Sorter;
import org.firstinspires.ftc.teamcode.gobot.TimingOptimization;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.gobot.VisionDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import android.util.Size;

// ------  TODO !!! -------
// Make sure to "Pseudo-Cache" the Pinpoint follower, to avoid multiple i2c calls!
//@Override
//public void loop() {
//    // 1. Trigger the actual I2C read ONCE
//    follower.update();
//
//    // 2. Cache the result in a local variable (FAST - in RAM)
//    Pose currentRobotPose = follower.getPose();
//
//    // 3. Use that local variable for EVERYTHING else in the loop
//    sorter.checkPosition(currentRobotPose);
//    launcher.updateTarget(currentRobotPose);
//}



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

    private Follower follower;

    //Timers and flags
    private ElapsedTime taskTimer;
    double intakeWaitTime;
//    private ElapsedTime totalTimer;
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

    private final int AllianceColor;
//    private int AllianceColor = 1; // Blue is 1
//    private int AllianceColor = 2; // Red is 2
    private final int StartPosition;
//    private int StartPosition = 1; // Front is 1
//    private int StartPosition = 1; // Back is 2
    private int taskCount;

    public static Pose startingPose;  //See ExampleAuto to understand how to use this

    private Pose shootPose;
    double shootSpeedRPS;
    double rearSpeedRPS = 58.5;
    double frontSpeedRPS = 54;
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

//        //[8] Emergency endPose
//        // TRAJ_8,   //Move to end pose
//        pathA.add(
//                follower.pathBuilder()
//                        .addPath(new BezierLine(follower::getPose, endPose))
//                        .setLinearHeadingInterpolation(follower::getHeading, endPose.getHeading())
//                        .build()
//        );
    }

    private TimingOptimization timingSystem;
    List<LynxModule> allHubs;
    private Intake intake;
    private Sorter sorter;
    private Lifter lifter;
    private Launcher launcher;
    private EnPointe enpointe;
    Pose currentPose;
    double currentHeading;
    boolean followerBusy;
    LED redLED;
    private boolean lastRedLedState;
    private boolean desiredRedLedState;


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
        int currentBall = 0; // Set the pattern position to ball 0

        // In auto, we should always start in the new position.
//        // If there is a stored pose (starting from PP auto) then
//        // Get the pose from storage
//        if(!( PoseStorage.currentPose==null) ) {
//            // This will load the pose if we have already run the robot.
//            startingPose = PoseStorage.currentPose;
//        }

        // Define the field poses for shooting and parking based on alliance color
        if (AllianceColor == 1) {
            frontShootPose = new Pose(59,14, Math.toRadians(115));
            rearShootPose = new Pose(58,85, Math.toRadians(135));
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
            frontShootPose = new Pose(86,14, Math.toRadians(67)); // Math.toRadians(66.5));
            rearShootPose = new Pose(88,85, Math.toRadians(49)); // Math.toRadians(180-133)); 47 deg
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
                scanAprilTagPose = new Pose(58.5, 25, Math.toRadians(83));
            } else if (StartPosition == 2) {
                scanAprilTagPose = new Pose(55, 110, Math.toRadians(67));
            }
            intakeLineup1Pose = new Pose(42, 36, Math.toRadians(180));
            intake1Ball1GPose = new Pose(36, 36, Math.toRadians(180));
            intake1Ball2PPose = new Pose(31, 36, Math.toRadians(180));
            intake1Ball3PPose = new Pose(26, 36, Math.toRadians(180));
            intakeLineup2Pose = new Pose(42, 60, Math.toRadians(180));
            intake2Ball1PPose = new Pose(36, 60, Math.toRadians(180));
            intake2Ball2GPose = new Pose(31, 60, Math.toRadians(180));
            intake2Ball3PPose = new Pose(26, 60, Math.toRadians(180));
            intakeLineup3Pose = new Pose(42, 84, Math.toRadians(180));
            intake3Ball1PPose = new Pose(36, 84, Math.toRadians(180));
            intake3Ball2PPose = new Pose(31, 84, Math.toRadians(180));
            intake3Ball3GPose = new Pose(26, 84, Math.toRadians(180));

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
                scanAprilTagPose = new Pose(85.5, 25, Math.toRadians(97));
            } else if (StartPosition == 2) {
                scanAprilTagPose = new Pose(90, 110, Math.toRadians(113));
            }
            intakeLineup1Pose = new Pose(102, 35, Math.toRadians(0));
            intake1Ball1GPose = new Pose(108, 35, Math.toRadians(0));
            intake1Ball2PPose = new Pose(113, 35, Math.toRadians(0));
            intake1Ball3PPose = new Pose(118, 35, Math.toRadians(0));
            intakeLineup2Pose = new Pose(102, 59, Math.toRadians(0));
            intake2Ball1PPose = new Pose(108, 59, Math.toRadians(0));
            intake2Ball2GPose = new Pose(113, 59, Math.toRadians(0));
            intake2Ball3PPose = new Pose(118, 59, Math.toRadians(0));
            intakeLineup3Pose = new Pose(102, 83, Math.toRadians(0));
            intake3Ball1PPose = new Pose(108, 83, Math.toRadians(0));
            intake3Ball2PPose = new Pose(113, 83, Math.toRadians(0));
            intake3Ball3GPose = new Pose(118, 83, Math.toRadians(0));

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
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        timingSystem = new TimingOptimization(telemetry);
        timingSystem.init();

        redLED = hardwareMap.get(LED.class, "lockdown_LED1");
        redLED.enable(false);
        lastRedLedState = false;

        intake = new Intake();
        intake.init(hardwareMap);
        lifter = new Lifter();
        lifter.init(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        sorter.init(lifter);
        launcher = new Launcher();
        launcher.init(hardwareMap);
        taskCount = 0;
        enpointe = new EnPointe();

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
        intakeWaitTime = 1;
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
        for (LynxModule hub : allHubs) { hub.clearBulkCache(); }
        // 1. THE PULSE: Update follower and cache the pose immediately.
        // This ensures every subsystem uses the same coordinate snapshot for this loop.
        follower.update();
        currentPose = follower.getPose();
        currentHeading = follower.getHeading();
        followerBusy = follower.isBusy();

        PoseStorage.currentPose = currentPose;

        timingSystem.update();

        // 2. SUBSYSTEM UPDATES
        sorter.update();
        lifter.update();
        launcher.update();

        // 3. PATH LOGIC
        // We pass the cached pose to updatePath to avoid redundant getPose() calls inside the switch.
        updatePath();

        // 4. OPTIMIZED TELEMETRY (Using your timingSystem)
        if (timingSystem.do_telemetry()) {
            telemetry.addData("Path State", currentState);
            telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.2f",
                    currentPose.getX(),
                    currentPose.getY(),
                    currentPose.getHeading());
            telemetry.addData("Launcher", "Setpoint: %.1f | Actual: %.1f RPS",
                    launcher.getActiveSetpointRPS(),
                    launcher.actual_RPS);
            telemetry.addData("Launcher Nominal", "%.1f RPS", launcher.getNominalRPS());
            // Subsystem-specific telemetry
            this.sorter.balls.telemetry();

            // Single update call to push the buffer to the Driver Station
            telemetry.update();
        }
    }

    public void updatePath() {
        // --- LED State Management (Bus-Safe) ---
        // Red LED indicates the autonomous program is actively processing a trajectory or task
        desiredRedLedState = (currentState != State.IDLE && currentState != State.STOP);
        if (desiredRedLedState != lastRedLedState) {
            redLED.enable(desiredRedLedState);
            lastRedLedState = desiredRedLedState;
        }

        switch (currentState) {
            case IDLE:
                currentState = State.TRAJ_0;
                break;

            case TRAJ_0:
                // Use cached followerBusy to avoid extra method overhead
                if (!followerBusy) {
                    launcher.setNominalRPS(shootSpeedRPS);
                    launcher.enableMotor();
                    follower.followPath(pathA.get(0), true);
                    currentState = State.SCAN;
                    taskState = TaskState.TASK_IDLE;
                }
                break;

            case SCAN:
                if (!followerBusy) {
                    if (taskState == TaskState.TASK_IDLE) {
                        taskTimer.reset();
                        taskState = TaskState.TASK_SCANNING;
                    }

                    patternID = vision.getDetectedPatternID();
                    // If we find it, or time out after 1 second
                    if (patternID != 0 || taskTimer.milliseconds() > 1000) {
                        if (patternID == 0) {
                            patternID = 22;
                            colorPattern = Arrays.asList('P', 'G', 'P');
                        } else {
                            colorPattern = vision.getArtifactColorPattern();
                        }
                        this.sorter.balls.setPatternList(colorPattern);
                        currentState = State.TRAJ_1;
                        taskState = TaskState.TASK_IDLE;
                    }
                }
                break;

            case TRAJ_1:
                if (!followerBusy) {
                    follower.setMaxPower(0.8);
                    follower.followPath(pathA.get(1), true);
                    currentState = State.SHOOT_A;
                }
                break;

            case SHOOT_A:
            case SHOOT_B:
                if (!followerBusy) {
                    this.sorter.balls.updateColors();

                    switch (taskState) {
                        case TASK_IDLE:
                            if (this.sorter.balls.targetMatchForLaunch()) {
                                if (!sorter.isBusy() && launcher.isReady()) {
                                    lifter.start();
                                    taskState = TaskState.TASK_LIFTING;
                                }
                            } else if (this.sorter.balls.anyBallAvailable()) {
                                // If target color isn't ready, but balls exist, sort.
                                if (!lifter.isBusy() && !sorter.isBusy()) {
                                    sorter.start(1);
                                    taskState = TaskState.TASK_SORTING;
                                }
                            } else {
                                // No balls left in sorter: Transition out
                                taskState = TaskState.TASK_IDLE;
                                launcher.disableMotor();
                                follower.setMaxPower(1.0);
                                if (currentState == State.SHOOT_A) {
                                    intake.setIntake(5); // Start intake rollers early here!
                                    currentState = State.TRAJ_2;
                                } else {
                                    currentState = State.TRAJ_7;
                                }
                            }
                            break;

                        case TASK_LIFTING:
                            if (!lifter.isBusy()) taskState = TaskState.TASK_IDLE;
                            break;

                        case TASK_SORTING:
                            if (!sorter.isBusy()) taskState = TaskState.TASK_IDLE;
                            break;
                    }
                }
                break;

            case TRAJ_2:
                if (!followerBusy) {
                    // intake.setIntake(5);
                    follower.followPath(pathA.get(2), true);
                    currentState = State.TRAJ_3;
                }
                break;

            case TRAJ_3:
            case TRAJ_4:
            case TRAJ_5:
                if (!followerBusy) {
                    switch (taskState) {
                        case TASK_IDLE:
                            if (!lifter.isBusy()) {
                                sorter.start(1);
                                intake.setIntake(-2); // Burst reverse to help ball seating
                                taskState = TaskState.TASK_SORTING;
                            }
                            break;
                        case TASK_SORTING:
                            if (!sorter.isBusy()) {
                                intake.setIntake(5);
                                follower.setMaxPower(0.2);
                                // Map the current trajectory to the path index
                                int pathIndex = (currentState == State.TRAJ_3) ? 3 : (currentState == State.TRAJ_4 ? 4 : 5);
                                follower.followPath(pathA.get(pathIndex), true);

                                taskState = TaskState.TASK_IDLE;
                                taskTimer.reset();
                                // Advance state
                                if (currentState == State.TRAJ_3) currentState = State.TRAJ_4;
                                else if (currentState == State.TRAJ_4) currentState = State.TRAJ_5;
                                else currentState = State.TRAJ_6;
                            }
                            break;
                    }
                }
                break;

            case TRAJ_6:
                // Wait for the final intake pull-in time we set in TRAJ_5
                // Intake wait time -> was 1.5. I'm setting it to 1
                if (!followerBusy && taskTimer.time() > intakeWaitTime) {
                    intake.setIntake(0);
                    launcher.enableMotor();
                    follower.setMaxPower(1.0);
                    follower.followPath(pathA.get(6), true);
                    currentState = State.SHOOT_B;
                }
                break;

            case TRAJ_7:
                if (!followerBusy) {
                    follower.followPath(pathA.get(7), true);
                    currentState = State.STOP;
                }
                break;

            case STOP:
                launcher.disableMotor();
                intake.setIntake(0);
                // Program Finished
                break;
        }
    }
}
