package org.firstinspires.ftc.teamcode.gobot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import android.util.Size;

// Using the standard, external camera control interfaces
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Utility class for managing AprilTag detection using a Logitech C920 camera,
 * specifically targeting the 2025-2026 FTC Decode artifact color pattern identification.
 */
public class VisionDetection {

    // --- Alliance and Target Goal IDs ---
    private static final int BLUE_ALLIANCE = 1;
    private static final int RED_ALLIANCE = 2;
    private static final int BLUE_GOAL_ID = 20; // Requested Blue Goal ID
    private static final int RED_GOAL_ID = 24;  // Requested Red Goal ID

    // --- Artifact Color Pattern IDs (2025-2026 Decode) ---
    private static final int PATTERN_LEFT_ID = 21;
    private static final int PATTERN_CENTER_ID = 22;
    private static final int PATTERN_RIGHT_ID = 23;

    // --- Vision Components ---
    private final AprilTagProcessor aprilTagProcessor;
    AprilTagLibrary aprilTagLibrary;
    private final VisionPortal visionPortal;

    // Alliance-specific field for Pedropathing
    private final int targetGoalId;

    // --- New Member for Stored Pattern ---
    // Stores the detected pattern ID (21, 22, 23) once it is found. Initializes to 0 (not found).
    private int detectedPatternID = 0;

    // --- Configuration Members ---
    private final boolean manualConfig;
    private final Size resolution;
    private ExposureControl exposureControl;
    private int exposureMs;
    private int     minExposure ;
    private int     maxExposure ;
    private int gain;
    private int     minGain ;
    private int     maxGain ;

    /**
     * Initializes the vision system with default (auto) camera settings.
     * @param hardwareMap The robot's hardware map (required to get the camera).
     * @param teamColor The alliance color (1 for Blue, 2 for Red).
     */
    public VisionDetection(HardwareMap hardwareMap, int teamColor) {
        // Default to auto settings
        this(hardwareMap, teamColor, false, new Size(640, 480), 10, 255);
    }

    /**
     * Initializes the vision system and sets the alliance-specific target goal,
     * optionally applying manual camera configuration.
     * * @param hardwareMap The robot's hardware map (required to get the camera).
     * @param teamColor The alliance color (1 for Blue, 2 for Red).
     * @param manualConfig If true, applies manual settings for resolution, exposure, and gain.
     * @param resolution The manual resolution to set (e.g., new Size(640, 480)).
     * @param exposureMs Manual exposure duration in milliseconds (e.g., 6-15 ms).
     * @param gain Manual gain value (e.g., 50-255).
     */
    public VisionDetection(HardwareMap hardwareMap, int teamColor,
                           boolean manualConfig, Size resolution,
                           int exposureMs, int gain) {

        this.manualConfig = manualConfig;
        this.resolution = resolution;
        this.exposureMs = exposureMs;
        this.gain = gain;

        // 1. Determine the target goal ID based on the alliance color
        if (teamColor == BLUE_ALLIANCE) {
            this.targetGoalId = BLUE_GOAL_ID;
        } else // (teamColor == RED_ALLIANCE)
        {
            this.targetGoalId = RED_GOAL_ID;
        }
//        } else {
//            // Handle invalid input by defaulting or throwing an exception
//            System.err.println("Invalid teamColor input. Defaulting to Blue Alliance (ID " + BLUE_GOAL_ID + ").");
//            this.targetGoalId = BLUE_GOAL_ID;
//        }

        // 2. Create the AprilTag processor
        AprilTagProcessor.Builder processorBuilder = new AprilTagProcessor.Builder();

        aprilTagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        // Use the official library directly, assuming all game tags are included
        processorBuilder.setTagLibrary(aprilTagLibrary);

        // --- Added drawing features to display tags in the live view ---
        processorBuilder.setDrawTagID(true);
        processorBuilder.setDrawTagOutline(true);
        processorBuilder.setDrawAxes(true);
        processorBuilder.setDrawCubeProjection(true);
        // ----------------------------------------------------------------

        aprilTagProcessor = processorBuilder.build();

        // 3. Build the Vision Portal
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Use the Logitech C920 camera
                .addProcessor(aprilTagProcessor);

        // 4. Apply manual configuration if requested
        if (manualConfig) {
            // Set the desired resolution
            builder.setCameraResolution(resolution);
        }

        // 5. Build and open the vision portal
        visionPortal = builder.build();

        // 6. Manual settings check: Advise user to call applyManualCameraSettings() later.
        if (manualConfig) {
            /* * CRITICAL NOTE ON MANUAL CAMERA CONTROL:
             * The camera stream must be ready before setting exposure/gain. These settings
             * should be applied *after* checking that visionPortal.getCameraState()
             * is VisionPortal.CameraState.STREAMING, typically in the OpMode's initialization
             * loop, NOT in the constructor. The helper method `applyManualCameraSettings()`
             * is provided for this purpose.
             */
        }
    }

    /*
    Manually set the camera gain and exposure.
    Can only be called AFTER calling initAprilTag();
    Returns true if controls are set.
 */
    private boolean    setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

//        // Wait for the camera to be open
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }

        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
//            sleep(20);
            return (true);
//        } else {
//            return (false);
//        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

//        // Wait for the camera to be open
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }

        // Get camera control values unless we are stopping.
//        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
//        }
    }

//    /**
//     * Helper method to safely apply the stored manual exposure and gain settings.
//     * This should typically be called from the OpMode's initialization routine
//     * after ensuring the VisionPortal is fully streaming.
//     */
//    public void applyManualCameraSettings() {
//        if (!manualConfig) {
//            System.out.println("Manual configuration was not enabled. Skipping settings application.");
//            return; // Only apply if manual config was requested in the constructor
//        }
//
//        // Ensure the controls are available and the camera is streaming
//        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
//
//  //            // Access the ExposureControl and GainControl interfaces via the VisionPortal
//  //            ExposureControl exposureControl = visionPortal.getExposureControl();
//
//
//            exposureControl = vuforia.getCamera().getControl(ExposureControl.class);
//
//
//  //            GainControl gainControl = visionPortal.getGainControl();
//
//            // Set Exposure
//            if (exposureControl.isExposureSupported() && exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
//
//
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//
//            } else {
//                System.err.println("Manual exposure not supported or failed to set.");
//            }
//
//            // Set Gain
//            if (gainControl.isGainSupported()) {
//                gainControl.setGain(gain);
//            } else {
//                System.err.println("Gain control not supported or failed to set.");
//            }
//        } else {
//            System.err.println("Camera not streaming. Manual settings cannot be applied yet.");
//        }
//    }

    // --- New Getter/Setter Methods for Exposure and Gain ---

    /**
     * Gets the current target exposure time in milliseconds.
     * NOTE: This returns the *configured* value, not the camera's *actual* value.
     * @return The configured exposure time in ms.
     */
    public int getManualExposureMs() {
        return this.exposureMs;
    }

//    /**
//     * Sets a new target exposure time (in ms) and attempts to apply it to the camera.
//     * @param exposureMs The new exposure time in milliseconds.
//     */
//    public void setManualExposureMs(int exposureMs) {
//        this.exposureMs = exposureMs;
//        // Attempt to apply the change immediately if manual configuration is enabled
//        if (manualConfig) {
//            ExposureControl exposureControl = visionPortal.getExposureControl();
//            if (exposureControl.isExposureSupported() && exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
//            } else {
//                System.err.println("Failed to set manual exposure on the fly. Check if streaming and supported.");
//            }
//        }
//    }

    /**
     * Gets the current target gain value.
     * NOTE: This returns the *configured* value, not the camera's *actual* value.
     * @return The configured gain value.
     */
    public int getManualGain() {
        return this.gain;
    }

//    /**
//     * Sets a new target gain value and attempts to apply it to the camera.
//     * @param gain The new gain value.
//     */
//    public void setManualGain(int gain) {
//        this.gain = gain;
//        // Attempt to apply the change immediately if manual configuration is enabled
//        if (manualConfig) {
//            GainControl gainControl = visionPortal.getGainControl();
//            if (gainControl.isGainSupported()) {
//                gainControl.setGain(gain);
//            } else {
//                System.err.println("Failed to set gain on the fly. Check if supported.");
//            }
//        }
//    }

    /*
     * =========================================================================================
     * VALID CAMERA OPTIONS FOR LOGITECH C920 (Based on FTC Documentation)
     * =========================================================================================
     * * 1. RESOLUTION (Size):
     * - COMMON CHOICES: new Size(640, 480) [Default/Good], new Size(640, 360),
     * new Size(800, 448), new Size(800, 600), new Size(864, 480), new Size(1920, 1080).
     * - PERFORMANCE NOTE: Lower resolutions (like 640x480 or 640x360) require less CPU
     * power and result in faster vision processing frame rates.
     * - CALIBRATION NOTE: Built-in calibration values exist for 640x480, 640x360, 800x448,
     * 800x600, 864x480, and 1920x1080. It is recommended to use one of these for AprilTag accuracy.
     * * 2. EXPOSURE (ExposureControl):
     * - MODE: Must be set to ExposureControl.Mode.Manual.
     * - RANGE (C920): Approximately 0 to 204 milliseconds (ms).
     * - OPTIMAL SETTING: To minimize motion blur while the robot is moving, set the exposure
     * duration to a small value, typically between 6 ms and 15 ms. Shorter exposure means less light.
     * * 3. GAIN (GainControl):
     * - RANGE (C920): Approximately 0 to 255 (unitless amplification).
     * - OPTIMAL SETTING: Gain boosts the image signal (making it brighter) but also adds noise.
     * Use the shortest exposure first, and then increase the gain (e.g., 50 to 255) until
     * reliable detection is achieved.
     * * =========================================================================================
     * COMPETITION OPTIMIZATION STRATEGY
     * =========================================================================================
     * * 1. SET MANUAL MODE: Always use `manualConfig = true` in competition. Arena lighting
     * is highly variable (fluorescent, LED, sunlight) and auto settings can cause the image
     * to flicker or suddenly change exposure mid-match, leading to detection failures.
     * 2. FIND OPTIMAL EXPOSURE: Start with a fast exposure (e.g., 10 ms). This reduces blur
     * when the robot is moving.
     * 3. ADJUST GAIN: If the image is too dark at the desired fast exposure, slowly increase
     * the gain value until the AprilTags are reliably detected from your maximum operational
     * distance (e.g., 12-24 inches).
     * 4. TEST IN DIFFERENT LIGHTING: Ideally, test your optimal settings under different
     * lighting conditions (e.g., different sides of the field, different arenas) to find
     * a robust middle ground.
     */

    /**
     * Retrieves the AprilTag ID (21, 22, or 23) corresponding to the artifact color pattern
     * currently visible to the camera. This result is cached, so once a pattern is detected,
     * subsequent calls will return the cached ID without further processing.
     * * * @return The detected pattern AprilTag ID, or 0 if none of the target patterns are found.
     */
    public int getDetectedPatternID() {
        // If the pattern has already been successfully detected and stored, return the cached value.
        if (this.detectedPatternID != 0) {
            return this.detectedPatternID;
        }

        // Check if the camera stream is ready before trying to process detections
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return 0;
        }

        // Get the list of detections from the AprilTag processor
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            int id = detection.id;

            // Check only for the artifact color pattern tags (21, 22, 23)
            if (id == PATTERN_LEFT_ID || id == PATTERN_CENTER_ID || id == PATTERN_RIGHT_ID) {
                // Store the detected pattern ID before returning
                this.detectedPatternID = id;
                return id;
            }
        }

        return 0; // Return 0 if no relevant pattern tag is detected
    }

    /**
     * Retrieves the stored artifact color pattern ID. This ID is cached the first time
     * a pattern tag (21, 22, or 23) is successfully detected by the camera via
     * the {@code getDetectedPatternID()} method.
     *
     * @return The cached pattern AprilTag ID (21, 22, 23), or 0 if nothing has been detected yet.
     */
    public int getStoredPatternID() {
        return this.detectedPatternID;
    }

    /**
     * Returns the target goal AprilTag ID for the current alliance (20 or 24).
     * This is commonly used in Pedropathing for setting a navigation target.
     * * @return The target goal ID (20 for Blue, 24 for Red).
     */
    public int getTargetGoalId() {
        return targetGoalId;
    }

    /**
     * Returns the associated artifact color pattern as a list of characters
     * based on the detected pattern ID (21, 22, or 23).
     * 'G' is Green (Artifact), 'P' is Purple (Artifact).
     * * @return A List<Character> representing the color pattern (e.g., ['G', 'P', 'P']),
     * or an empty list if no pattern is detected.
     */
    public List<Character> getArtifactColorPattern() {
        // We use getDetectedPatternID() here to ensure detection is attempted
        // if the pattern hasn't been cached yet.
        int patternID = getDetectedPatternID();

        // The specific color patterns are based on the 2025-2026 FTC Decode artifacts
        switch (patternID) {
            case PATTERN_LEFT_ID: // ID 21: Green, Purple, Purple
                return Arrays.asList('G', 'P', 'P');
            case PATTERN_CENTER_ID: // ID 22: Purple, Green, Purple
                return Arrays.asList('P', 'G', 'P');
            case PATTERN_RIGHT_ID: // ID 23: Purple, Purple, Green
                return Arrays.asList('P', 'P', 'G');
            default:
                // If the pattern tag is not found (ID 0), return an empty list
                return new ArrayList<>();
        }
    }

    /**
     * Searches for the alliance's target goal AprilTag and returns its pose data.
     *
     * @return A double array containing [range, bearing, elevation] to the target goal,
     * or null if the target tag is not currently detected or the camera stream is not ready.
     * The indices are: 0=range (in inches), 1=bearing (in degrees), 2=elevation (in degrees).
     */
    public double[] getTargetGoalPose() {
        // Ensure the camera stream is active
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return null;
        }

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            // Check if the detected ID matches the alliance's target goal ID (20 or 24)
            if (detection.id == this.targetGoalId) {
                // Check if pose data is available for this detection
                if (detection.ftcPose != null) {
                    // Return the pose data in the specified format [range, bearing, elevation]
                    return new double[]{
                            detection.ftcPose.range,
                            detection.ftcPose.bearing,
                            detection.ftcPose.elevation
                    };
                }
            }
        }

        return null; // Target tag not detected or pose data unavailable
    }

    /**
     * Closes the VisionPortal, which automatically stops the live camera view
     * and releases camera resources.
     * (e.g., call this in the OpMode's stop() method or when vision is no longer needed).
     */
    public void closeVision() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}