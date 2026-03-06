package frc.robot.subsystems.intake;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * AprilTag vision subsystem for the Luma P1 running PhotonVision.
 *
 * Hardware notes (Luma P1):
 *   - OV9281 global shutter sensor — eliminates rolling-shutter motion blur
 *   - 1280x800 resolution at up to 120 fps
 *   - 80 degree horizontal / 56 degree vertical FOV
 *   - Runs PhotonVision on Raspberry Pi CM5
 *   - Access UI at photonvision.local:5800
 *
 * 2026 REBUILT field layout notes:
 *   - 32 unique AprilTags, IDs 1-32, all from the 36h11 family
 *   - Tag size: 8.125 in (20.64 cm) square
 *   - HUB tags  : IDs 2-5, 8-11, 18-21, 24-27  (centers 44.25 in off floor)
 *   - Tower tags: IDs 13-16, 29-32              (centers 21.75 in off floor)
 *   - Trench tags: IDs 1, 6, 7, 12, 17, 22, 23, 28 (centers 35 in off floor)
 *   - TWO field variants exist: welded (PhotonVision default) and AndyMark.
 *     The JSON deployed here MUST match the layout uploaded to your Luma P1.
 *
 * Key design decisions:
 *   - Extends SubsystemBase so periodic() is called every 20 ms automatically
 *   - Uses getAllUnreadResults() (replaces deprecated getLatestResult())
 *     called EXACTLY ONCE per loop in periodic(), then caches for all callers
 *   - PhotonPoseEstimator uses the new 2-arg constructor with individual
 *     estimateCoprocMultiTagPose / estimateLowestAmbiguityPose calls
 *     (old 3-arg strategy constructor is deprecated for removal in 2026)
 *   - Std-dev matrix scales with tag distance: farther = less trust
 *   - All pose estimates are sanity-checked against field boundary + ambiguity
 */
public class AprilTag extends SubsystemBase {
    private static final boolean PUBLISH_VISION_TELEMETRY = false;
    private static final int VISION_POLL_DIVISOR = 5; // 100 ms at 20 ms robot loop
    private static final long VISION_RETRY_DELAY_MS = 1000;

    // -----------------------------------------------------------------------
    // Camera name — must match exactly what is set in the PhotonVision UI
    // on the Luma P1 (navigate to photonvision.local:5800 to verify/change)
    // -----------------------------------------------------------------------
    public static final String CAMERA_NAME = "OV9281";
    private static final String LUMA_STREAM_NAME = "LumaP1";
    private static final String[] LUMA_STREAM_URLS = {
            "http://photonvision.local:1182/?action=stream",
            "http://photonvision.local:1181/?action=stream"
    };

    // -----------------------------------------------------------------------
    // Robot-to-camera transform (robot center at floor level → camera lens)
    //
    // FRC robot coordinate system: +X = forward, +Y = left, +Z = up
    //
    // IMPORTANT: Measure and adjust these values to match your actual P1
    // mounting position. Wrong values will produce wrong pose estimates.
    //
    // Example below: camera 30 cm forward of center, 50 cm above floor,
    // tilted 15 degrees downward so it can see floor-level tags.
    // -----------------------------------------------------------------------
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            new Translation3d(0.30, 0.0, 0.50),
            new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)
    );

    // -----------------------------------------------------------------------
    // Pose-estimation trust tuning (units: meters, meters, radians)
    // Lower value = trust this measurement MORE.
    // These are multiplied by a distance factor in getEstimationStdDevs().
    // Multi-tag is trusted much more than single-tag.
    // -----------------------------------------------------------------------
    private static final Matrix<N3, N1> MULTI_TAG_BASE_STD_DEVS  = VecBuilder.fill(0.3, 0.3, 0.5);
    private static final Matrix<N3, N1> SINGLE_TAG_BASE_STD_DEVS = VecBuilder.fill(1.5, 1.5, 3.0);

    // Reject single-tag estimates with pose ambiguity above this value.
    // 0.2 is the standard recommendation; lower = stricter.
    private static final double MAX_AMBIGUITY = 0.2;

    // Reject estimates where any used tag is farther than this.
    private static final double MAX_TAG_DISTANCE_METERS = 5.0;

    // 2026 field boundary (meters). Estimates outside this box are discarded.
    private static final double FIELD_LENGTH_METERS = 17.548;
    private static final double FIELD_WIDTH_METERS  = 8.210;

    // -----------------------------------------------------------------------
    // Field layout file paths
    //
    // Priority:
    //   1. Local deploy file  → put the JSON in src/main/deploy/ and it will
    //      be copied to /home/lvuser/deploy/ on the robot automatically.
    //      Download from: https://github.com/wpilibsuite/allwpilib (see path below)
    //   2. GitHub URL fallback — NO internet at competition, dev bench only
    //   3. WPILib bundled welded field — last resort
    //
    // REMINDER: the JSON here and the layout uploaded to the Luma P1 must match.
    // -----------------------------------------------------------------------
    private static final String LOCAL_DEPLOY_JSON =
            "/home/lvuser/deploy/2026-rebuilt-andymark.json";

    // Note: we no longer attempt to fetch the AndyMark JSON from GitHub at runtime.
    // Rely on the deploy file or an embedded resource instead to be competition-safe.
    private static final String CLASSPATH_RESOURCE = "/2026-rebuilt-andymark.json";

    // -----------------------------------------------------------------------
    // Internal state
    // -----------------------------------------------------------------------
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;

    // Cached from getAllUnreadResults() — updated once per periodic()
    private List<PhotonPipelineResult> currentResults = new ArrayList<>();

    // Most recent frame that contained at least one target
    private PhotonPipelineResult latestResultWithTargets = new PhotonPipelineResult();

    // Latest accepted pose estimate and its std devs
    private Optional<EstimatedRobotPose> latestPoseEstimate = Optional.empty();
    private Matrix<N3, N1> latestStdDevs = SINGLE_TAG_BASE_STD_DEVS;
    private HttpCamera lumaStreamCamera;
    private boolean lumaStreamPublished = false;
    private int pollTick = 0;
    private long nextVisionRetryMs = 0;
    private boolean warnedVisionUnavailable = false;

    // -----------------------------------------------------------------------
    // Constructors
    // -----------------------------------------------------------------------
    public AprilTag() {
        this(CAMERA_NAME);
    }

    public AprilTag(String cameraName) {
        fieldLayout = loadFieldLayout();
        camera = new PhotonCamera(cameraName);

        // 2-argument constructor (new API for 2026 — strategy passed per-call)
        poseEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_CAMERA);
    }

    // -----------------------------------------------------------------------
    // SubsystemBase periodic — called every 20 ms by the CommandScheduler
    // -----------------------------------------------------------------------
    @Override
    public void periodic() {
        if ((pollTick++ % VISION_POLL_DIVISOR) != 0) {
            return;
        }

        long nowMs = System.currentTimeMillis();
        if (nowMs < nextVisionRetryMs) {
            return;
        }

        // Call getAllUnreadResults() EXACTLY ONCE per loop.
        // It drains the internal FIFO — calling it again returns empty/stale data.
        try {
            currentResults = camera.getAllUnreadResults();
            warnedVisionUnavailable = false;
        } catch (Exception e) {
            currentResults = new ArrayList<>();
            latestPoseEstimate = Optional.empty();
            nextVisionRetryMs = nowMs + VISION_RETRY_DELAY_MS;
            if (!warnedVisionUnavailable) {
                DriverStation.reportWarning(
                        "AprilTag vision temporarily disabled: camera '" + CAMERA_NAME
                                + "' not available. Retrying periodically.",
                        false);
                warnedVisionUnavailable = true;
            }
            return;
        }

        // Cache the most recent frame that had targets
        for (var result : currentResults) {
            if (result.hasTargets()) {
                latestResultWithTargets = result;
            }
        }

        // Run pose estimation over all new frames; keep the last accepted one
        latestPoseEstimate = Optional.empty();
        for (var result : currentResults) {
            if (!result.hasTargets()) continue;

            // Try multi-tag PNP first (computed on the P1 coprocessor — best accuracy)
            Optional<EstimatedRobotPose> est =
                    poseEstimator.estimateCoprocMultiTagPose(result);

            // Fall back to single-tag lowest-ambiguity if multi-tag unavailable
            if (est.isEmpty()) {
                est = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            if (est.isPresent() && isEstimateValid(est.get())) {
                latestPoseEstimate = est;
                latestStdDevs = getEstimationStdDevs(est.get());
            }
        }

        publishTelemetry();
        ensureLiveFeedPublished();
    }

    // -----------------------------------------------------------------------
    // Pose estimation — primary interface for SwerveSubsystem
    // -----------------------------------------------------------------------

    /**
     * Returns the latest accepted robot pose estimate from this loop, or empty
     * if no good estimate was produced (no targets, poor geometry, out of bounds).
     *
     * Usage in SwerveSubsystem:
     *   aprilTag.getLatestEstimate().ifPresent(est ->
     *       swervePoseEstimator.addVisionMeasurement(
     *           est.estimatedPose.toPose2d(),
     *           est.timestampSeconds,
     *           aprilTag.getLatestStdDevs()));
     */
    public Optional<EstimatedRobotPose> getLatestEstimate() {
        return latestPoseEstimate;
    }

    /**
     * Standard deviations for the latest accepted estimate.
     * Pass this to SwerveDrivePoseEstimator.addVisionMeasurement().
     */
    public Matrix<N3, N1> getLatestStdDevs() {
        return latestStdDevs;
    }

    /**
     * Compute distance-scaled standard deviations for a given estimate.
     * Confidence decreases as the robot is farther from the tags.
     */
    public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimate) {
        List<PhotonTrackedTarget> usedTargets = estimate.targetsUsed;
        int numTags = usedTargets.size();

        if (numTags == 0) return SINGLE_TAG_BASE_STD_DEVS;

        double avgDistMeters = usedTargets.stream()
                .mapToDouble(t -> {
                    Transform3d c2t = t.getBestCameraToTarget();
                    return Math.hypot(c2t.getX(), c2t.getY());
                })
                .average()
                .orElse(Double.MAX_VALUE);

        // Scale quadratically with distance: farther tags = less trust
        double distFactor = 1.0 + (avgDistMeters * avgDistMeters) * 0.08;
        Matrix<N3, N1> base =
                (numTags >= 2) ? MULTI_TAG_BASE_STD_DEVS : SINGLE_TAG_BASE_STD_DEVS;
        return base.times(distFactor);
    }

    // -----------------------------------------------------------------------
    // Target queries — use cached results from this loop
    // -----------------------------------------------------------------------

    /**
     * Get the tracked target for a specific tag ID from the latest frame,
     * if visible. Prefers the target with the largest area (closest).
     */
    public Optional<PhotonTrackedTarget> getLatestTrackedTarget(int tagId) {
        return latestResultWithTargets.getTargets().stream()
                .filter(t -> t.getFiducialId() == tagId)
                .max(Comparator.comparingDouble(PhotonTrackedTarget::getArea));
    }

    /**
     * Returns the best (largest area) target from the latest frame.
     */
    public Optional<PhotonTrackedTarget> getBestVisibleTarget() {
        if (!latestResultWithTargets.hasTargets()) return Optional.empty();
        return Optional.of(latestResultWithTargets.getBestTarget());
    }

    /**
     * Returns the camera-to-tag Transform3d for a given tag ID.
     * Used by Alignment.java for direct vision-based driving.
     */
    public Optional<Transform3d> getCameraToTagTransform(int tagId) {
        return getLatestTrackedTarget(tagId)
                .map(PhotonTrackedTarget::getBestCameraToTarget);
    }

    /** True if the given tag ID is currently visible to the camera. */
    public boolean seesTag(int tagId) {
        return getLatestTrackedTarget(tagId).isPresent();
    }

    /** True if the Luma P1 is connected and sending data over NetworkTables. */
    public boolean isCameraConnected() {
        return camera.isConnected();
    }

    // -----------------------------------------------------------------------
    // Field layout access
    // -----------------------------------------------------------------------

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    /**
     * Set the field origin for the current alliance color.
     * Call this at the start of autonomousInit() and teleopInit().
     */
    public void updateOriginFromAlliance(Optional<DriverStation.Alliance> alliance) {
        OriginPosition origin = (alliance.isPresent()
                && alliance.get() == DriverStation.Alliance.Red)
                ? OriginPosition.kRedAllianceWallRightSide
                : OriginPosition.kBlueAllianceWallRightSide;
        fieldLayout.setOrigin(origin);
        // Keep the estimator's copy in sync with the updated origin
        poseEstimator.setFieldTags(fieldLayout);
    }

    public Optional<Pose3d> getTagPose3d(int tagId) {
        return fieldLayout.getTagPose(tagId);
    }

    public Optional<Pose2d> getTagPose2d(int tagId) {
        return getTagPose3d(tagId).map(Pose3d::toPose2d);
    }

    // -----------------------------------------------------------------------
    // Per-tag telemetry (call from dashboard or vision logging routines)
    // -----------------------------------------------------------------------
    public void logVision(int tagId) {
        if (!PUBLISH_VISION_TELEMETRY) {
            return;
        }
        Optional<PhotonTrackedTarget> targetOpt = getLatestTrackedTarget(tagId);
        boolean visible = targetOpt.isPresent();
        SmartDashboard.putBoolean("Vision/Tag" + tagId + "/Visible", visible);
        if (visible) {
            PhotonTrackedTarget t = targetOpt.get();
            Transform3d c2t = t.getBestCameraToTarget();
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/YawDeg",    t.getYaw());
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/PitchDeg",  t.getPitch());
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/Area",      t.getArea());
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/Ambiguity", t.getPoseAmbiguity());
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/DistM",
                    Math.hypot(c2t.getX(), c2t.getY()));
        } else {
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/YawDeg",    0.0);
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/PitchDeg",  0.0);
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/Area",      0.0);
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/Ambiguity", 0.0);
            SmartDashboard.putNumber("Vision/Tag" + tagId + "/DistM",     0.0);
        }
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /**
     * Validate an estimated pose before accepting it:
     *   1. Robot position must be within field boundaries
     *   2. All used tags must be within MAX_TAG_DISTANCE_METERS
     *   3. Single-tag estimates must pass MAX_AMBIGUITY threshold
     */
    private boolean isEstimateValid(EstimatedRobotPose estimate) {
        Pose2d pose = estimate.estimatedPose.toPose2d();

        if (pose.getX() < 0 || pose.getX() > FIELD_LENGTH_METERS
                || pose.getY() < 0 || pose.getY() > FIELD_WIDTH_METERS) {
            return false;
        }

        List<PhotonTrackedTarget> targets = estimate.targetsUsed;

        boolean tooFar = targets.stream().anyMatch(t -> {
            Transform3d c2t = t.getBestCameraToTarget();
            return Math.hypot(c2t.getX(), c2t.getY()) > MAX_TAG_DISTANCE_METERS;
        });
        if (tooFar) return false;

        if (targets.size() == 1
                && targets.get(0).getPoseAmbiguity() > MAX_AMBIGUITY) {
            return false;
        }

        return true;
    }

    /** Publish camera health and vision state to SmartDashboard every loop. */
    private void publishTelemetry() {
        if (!PUBLISH_VISION_TELEMETRY) {
            return;
        }
        SmartDashboard.putBoolean("Vision/CameraConnected", camera.isConnected());
        SmartDashboard.putBoolean("Vision/LiveFeedPublished", lumaStreamPublished);
        SmartDashboard.putBoolean("Vision/HasTargets",
                latestResultWithTargets.hasTargets());
        SmartDashboard.putNumber("Vision/NumTargets",
                latestResultWithTargets.hasTargets()
                        ? latestResultWithTargets.getTargets().size() : 0);
        SmartDashboard.putBoolean("Vision/PoseEstimateValid",
                latestPoseEstimate.isPresent());

        if (latestResultWithTargets.hasTargets()) {
            SmartDashboard.putNumber("Vision/BestTagId",
                    latestResultWithTargets.getBestTarget().getFiducialId());
            SmartDashboard.putNumber("Vision/BestTagAmbiguity",
                    latestResultWithTargets.getBestTarget().getPoseAmbiguity());
        }

        latestPoseEstimate.ifPresent(est -> {
            Pose2d p = est.estimatedPose.toPose2d();
            SmartDashboard.putNumber("Vision/EstPoseX",    p.getX());
            SmartDashboard.putNumber("Vision/EstPoseY",    p.getY());
            SmartDashboard.putNumber("Vision/EstPoseDeg",  p.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/NumTagsUsed", est.targetsUsed.size());
        });
    }

    /**
     * Publish Luma P1 live feed to CameraServer once PhotonVision reports the camera connected.
     * This creates a stream entry that dashboards can display as a camera widget.
     */
    private void ensureLiveFeedPublished() {
        if (lumaStreamPublished || !camera.isConnected()) {
            return;
        }

        lumaStreamCamera = new HttpCamera(LUMA_STREAM_NAME, LUMA_STREAM_URLS);
        lumaStreamCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        CameraServer.addCamera(lumaStreamCamera);
        lumaStreamPublished = true;
        DriverStation.reportWarning("AprilTag: Luma connected, published live feed '" + LUMA_STREAM_NAME + "'.", false);
    }

    /**
     * Load the AprilTag field layout with a local-first fallback chain.
     *
     * CRITICAL for 2026: the welded and AndyMark field layouts have different
     * tag coordinates. Both your robot code AND the Luma P1's PhotonVision
     * settings must use the same variant or pose estimates will be wrong.
     *
     * To deploy the AndyMark JSON locally:
     *   1. Download 2026-rebuilt-andymark.json from WPILib's allwpilib repo
     *   2. Place it in src/main/deploy/
     *   3. Build and deploy — WPILib copies deploy/ to /home/lvuser/deploy/
     */
    private AprilTagFieldLayout loadFieldLayout() {
        // 1. Local file (preferred — works at competition with no internet)
        try {
            File f = new File(LOCAL_DEPLOY_JSON);
            if (f.exists()) {
                AprilTagFieldLayout layout =
                        new ObjectMapper().readValue(f, AprilTagFieldLayout.class);
                DriverStation.reportWarning(
                        "AprilTag: loaded AndyMark 2026 layout from local deploy file.", false);
                return layout;
            }
        } catch (IOException e) {
            DriverStation.reportWarning(
                    "AprilTag: local deploy JSON failed: " + e.getMessage(), false);
        }

        // 2. Try classpath resource (allows packaging the JSON into JAR/resources)
        try (InputStream is = AprilTag.class.getResourceAsStream(CLASSPATH_RESOURCE)) {
            if (is != null) {
                AprilTagFieldLayout layout = new ObjectMapper().readValue(is, AprilTagFieldLayout.class);
                DriverStation.reportWarning("AprilTag: loaded AndyMark 2026 layout from classpath resource.", false);
                return layout;
            }
        } catch (IOException e) {
            DriverStation.reportWarning("AprilTag: classpath resource load failed: " + e.getMessage(), false);
        }

        // 3. WPILib bundled layout — welded field variant, last resort
        return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
}
