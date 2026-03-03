package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.AprilTag;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Optional;
import java.util.Set;

public class BlueLeftAuto {
    private static final String PATH_FILE = "BlueLeftAuto";
    private static final int TARGET_TAG_ID = 23;

    // Requested hub-facing heading after path completion.
    private static final Rotation2d HUB_ROTATION = Rotation2d.fromDegrees(45.0);

    // Tunables for final align-to-shoot positioning near tag 7.
    private static final double SHOOT_STANDOFF_METERS = 1.20;
    private static final double TRANSLATION_KP = 1.5;
    private static final double ROTATION_KP = 2.4;
    private static final double MAX_TRANSLATION_MPS = 1.3;
    private static final double MAX_ROTATION_RAD_PER_SEC = 2.0;
    private static final double POSITION_TOLERANCE_METERS = 0.08;
    private static final double HEADING_TOLERANCE_DEG = 2.0;
    private static final double VISION_FORWARD_KP = 1.1;
    private static final double VISION_LATERAL_KP = 1.1;
    private static final double VISION_YAW_KP = 0.03;
    private static final double VISION_RANGE_TOLERANCE_METERS = 0.08;
    private static final double VISION_LATERAL_TOLERANCE_METERS = 0.06;
    private static final double VISION_YAW_TOLERANCE_DEG = 2.0;

    private BlueLeftAuto() {}

    public static Command build(SwerveSubsystem swerveSubsystem, AprilTag aprilTag, Shooter shooter) {
        return Commands.sequence(
                Commands.runOnce(() -> aprilTag.updateOriginFromAlliance(DriverStation.getAlliance())),
                followBlueLeftPath(swerveSubsystem),
                createTagAlignCommand(swerveSubsystem, aprilTag),
                rotateToHeading(swerveSubsystem, HUB_ROTATION),
                // Spin shooter at 50% power for 2 seconds, then stop
                Commands.runOnce(() -> shooter.spinAll(0.5), shooter),
                Commands.waitSeconds(2.0),
                Commands.runOnce(shooter::stop, shooter),
                Commands.runOnce(swerveSubsystem::stop, swerveSubsystem));
    }

    public static Command createTagAlignCommand(SwerveSubsystem swerveSubsystem, AprilTag aprilTag) {
        return alignToTagShootPose(swerveSubsystem, aprilTag, TARGET_TAG_ID);
    }

    /** Create an align-to-tag command for an arbitrary tag ID. */
    public static Command createTagAlignCommand(SwerveSubsystem swerveSubsystem, AprilTag aprilTag, int tagId) {
        return alignToTagShootPose(swerveSubsystem, aprilTag, tagId);
    }

    public static Command createScanAndRunTagCommand(SwerveSubsystem swerveSubsystem, AprilTag aprilTag) {
        final int[] detectedTagId = {-1};

        return Commands.sequence(
                Commands.runOnce(() -> {
                    detectedTagId[0] = -1;
                    aprilTag.updateOriginFromAlliance(DriverStation.getAlliance());
                }),
                Commands.run(() -> aprilTag.getBestVisibleTarget().ifPresent(target -> {
                            detectedTagId[0] = target.getFiducialId();
                            SmartDashboard.putNumber("Vision/DetectedTagId", detectedTagId[0]);
                        }))
                        .until(() -> detectedTagId[0] != -1)
                        .withTimeout(1.5),
                Commands.either(
                        rotateToHeading(swerveSubsystem, HUB_ROTATION),
                        Commands.none(),
                        () -> detectedTagId[0] == TARGET_TAG_ID),
                Commands.runOnce(swerveSubsystem::stop, swerveSubsystem));
    }

    private static Command followBlueLeftPath(SwerveSubsystem swerveSubsystem) {
        return Commands.defer(
                () -> {
                    try {
                        PathPlannerPath path = PathPlannerPath.fromPathFile(PATH_FILE);
                        return AutoBuilder.followPath(path);
                    } catch (Exception ex) {
                        DriverStation.reportWarning(
                                "BlueLeftAuto path load failed for '" + PATH_FILE + "': " + ex.getMessage(),
                                false);
                        return Commands.none();
                    }
                },
                Set.of(swerveSubsystem));
    }

    private static Command rotateToHeading(SwerveSubsystem swerveSubsystem, Rotation2d heading) {
        return Commands.run(
                        () -> {
                            double errorRad = headingErrorRad(swerveSubsystem.getPose().getRotation(), heading);
                            double omega = MathUtil.clamp(
                                    errorRad * ROTATION_KP,
                                    -MAX_ROTATION_RAD_PER_SEC,
                                    MAX_ROTATION_RAD_PER_SEC);
                            swerveSubsystem.driveFieldRelative(new ChassisSpeeds(0.0, 0.0, omega));
                        },
                        swerveSubsystem)
                .until(() -> atHeading(swerveSubsystem.getPose().getRotation(), heading))
                .withTimeout(1.5)
                .andThen(Commands.runOnce(swerveSubsystem::stop, swerveSubsystem));
    }

    private static Command alignToTagShootPose(SwerveSubsystem swerveSubsystem, AprilTag aprilTag, int tagId) {
        return Commands.run(
                        () -> {
                            aprilTag.logVision(tagId);
                            Optional<Transform3d> cameraToTagOptional = aprilTag.getCameraToTagTransform(tagId);

                            if (cameraToTagOptional.isPresent()) {
                                driveFromVisionMeasurement(swerveSubsystem, aprilTag, cameraToTagOptional.get(), tagId);
                                return;
                            }

                            Optional<Pose2d> tagPoseOptional = aprilTag.getTagPose2d(tagId);
                            if (tagPoseOptional.isEmpty()) {
                                swerveSubsystem.stop();
                                return;
                            }
                            Pose2d robotPose = swerveSubsystem.getPose();
                            Pose2d tagPose = tagPoseOptional.get();

                            Pose2d shootPose = computeShootPoseFromTag(tagPose);
                            double targetHeadingRad = Math.atan2(
                                    tagPose.getY() - robotPose.getY(),
                                    tagPose.getX() - robotPose.getX());
                            Rotation2d targetHeading = Rotation2d.fromRadians(targetHeadingRad);

                            double xError = shootPose.getX() - robotPose.getX();
                            double yError = shootPose.getY() - robotPose.getY();
                            double headingError = headingErrorRad(robotPose.getRotation(), targetHeading);

                            double vx = MathUtil.clamp(
                                    xError * TRANSLATION_KP,
                                    -MAX_TRANSLATION_MPS,
                                    MAX_TRANSLATION_MPS);
                            double vy = MathUtil.clamp(
                                    yError * TRANSLATION_KP,
                                    -MAX_TRANSLATION_MPS,
                                    MAX_TRANSLATION_MPS);
                            double omega = MathUtil.clamp(
                                    headingError * ROTATION_KP,
                                    -MAX_ROTATION_RAD_PER_SEC,
                                    MAX_ROTATION_RAD_PER_SEC);

                            swerveSubsystem.driveFieldRelative(new ChassisSpeeds(vx, vy, omega));
                        },
                        swerveSubsystem)
                .until(() -> atShootAlignmentTarget(swerveSubsystem, aprilTag, tagId))
                .withTimeout(2.0)
                .andThen(Commands.runOnce(swerveSubsystem::stop, swerveSubsystem));
    }

    private static void driveFromVisionMeasurement(
            SwerveSubsystem swerveSubsystem, AprilTag aprilTag, Transform3d cameraToTag) {
        driveFromVisionMeasurement(swerveSubsystem, aprilTag, cameraToTag, TARGET_TAG_ID);
    }

    private static void driveFromVisionMeasurement(
            SwerveSubsystem swerveSubsystem, AprilTag aprilTag, Transform3d cameraToTag, int tagId) {
        Optional<org.photonvision.targeting.PhotonTrackedTarget> trackedTarget =
                aprilTag.getLatestTrackedTarget(tagId);
        if (trackedTarget.isEmpty()) {
            swerveSubsystem.stop();
            return;
        }

        double rangeError = cameraToTag.getX() - SHOOT_STANDOFF_METERS;
        double lateralError = cameraToTag.getY();
        double yawErrorDeg = trackedTarget.get().getYaw();

    SmartDashboard.putNumber("Vision/Tag" + tagId + "/RangeErrorM", rangeError);
    SmartDashboard.putNumber("Vision/Tag" + tagId + "/LateralErrorM", lateralError);
    SmartDashboard.putNumber("Vision/Tag" + tagId + "/YawErrorDeg", yawErrorDeg);

        double vx = MathUtil.clamp(rangeError * VISION_FORWARD_KP, -MAX_TRANSLATION_MPS, MAX_TRANSLATION_MPS);
        double vy = MathUtil.clamp(lateralError * VISION_LATERAL_KP, -MAX_TRANSLATION_MPS, MAX_TRANSLATION_MPS);
        double omega = MathUtil.clamp(
                -Math.toRadians(yawErrorDeg) * (VISION_YAW_KP * 10.0),
                -MAX_ROTATION_RAD_PER_SEC,
                MAX_ROTATION_RAD_PER_SEC);

        swerveSubsystem.driveRobotRelative(new ChassisSpeeds(vx, vy, omega));
    }

    private static Pose2d computeShootPoseFromTag(Pose2d tagPose) {
        Rotation2d tagHeading = tagPose.getRotation();
        double x = tagPose.getX() - SHOOT_STANDOFF_METERS * tagHeading.getCos();
        double y = tagPose.getY() - SHOOT_STANDOFF_METERS * tagHeading.getSin();
        return new Pose2d(x, y, tagHeading);
    }

    private static boolean atShootAlignmentTarget(SwerveSubsystem swerveSubsystem, AprilTag aprilTag) {
        return atShootAlignmentTarget(swerveSubsystem, aprilTag, TARGET_TAG_ID);
    }

    private static boolean atShootAlignmentTarget(SwerveSubsystem swerveSubsystem, AprilTag aprilTag, int tagId) {
        Optional<Transform3d> cameraToTagOptional = aprilTag.getCameraToTagTransform(tagId);
        if (cameraToTagOptional.isPresent()) {
            Optional<org.photonvision.targeting.PhotonTrackedTarget> trackedTarget =
                    aprilTag.getLatestTrackedTarget(tagId);
            if (trackedTarget.isEmpty()) {
                return false;
            }

            Transform3d cameraToTag = cameraToTagOptional.get();
            double rangeError = Math.abs(cameraToTag.getX() - SHOOT_STANDOFF_METERS);
            double lateralError = Math.abs(cameraToTag.getY());
            double yawError = Math.abs(trackedTarget.get().getYaw());

            return rangeError <= VISION_RANGE_TOLERANCE_METERS
                    && lateralError <= VISION_LATERAL_TOLERANCE_METERS
                    && yawError <= VISION_YAW_TOLERANCE_DEG;
        }

        Optional<Pose2d> tagPoseOptional = aprilTag.getTagPose2d(tagId);
        if (tagPoseOptional.isEmpty()) {
            return false;
        }

        Pose2d robotPose = swerveSubsystem.getPose();
        Pose2d tagPose = tagPoseOptional.get();
        Pose2d shootPose = computeShootPoseFromTag(tagPose);

        double xError = shootPose.getX() - robotPose.getX();
        double yError = shootPose.getY() - robotPose.getY();
        double positionError = Math.hypot(xError, yError);

        Rotation2d faceTagHeading = Rotation2d.fromRadians(Math.atan2(
                tagPose.getY() - robotPose.getY(),
                tagPose.getX() - robotPose.getX()));

        return positionError <= POSITION_TOLERANCE_METERS
                && atHeading(robotPose.getRotation(), faceTagHeading);
    }

    private static boolean atHeading(Rotation2d current, Rotation2d target) {
        return Math.abs(headingErrorRad(current, target)) <= Math.toRadians(HEADING_TOLERANCE_DEG);
    }

    private static double headingErrorRad(Rotation2d current, Rotation2d target) {
        return MathUtil.angleModulus(target.getRadians() - current.getRadians());
    }
}
