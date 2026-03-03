package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.AprilTag;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Set;

public class BlueLeftAuto {
    private static final String PATH_FILE = "BlueLeftAuto";
    private static final int TARGET_TAG_ID = 23;

    // Requested hub-facing heading after path completion.
    private static final Rotation2d HUB_ROTATION = Rotation2d.fromDegrees(45.0);

    // Tunables used in autonomous path following/rotation
    private static final double ROTATION_KP = 2.4;
    private static final double MAX_ROTATION_RAD_PER_SEC = 2.0;
    private static final double HEADING_TOLERANCE_DEG = 2.0;

    private BlueLeftAuto() {}

    public static Command build(
            SwerveSubsystem swerveSubsystem,
            AprilTag aprilTag,
            Shooter shooter,
            frc.robot.subsystems.intake.PivotWheels pivotWheels,
            frc.robot.subsystems.intake.intake intake,
            frc.robot.subsystems.intake.hopper.hopper hopper) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    aprilTag.updateOriginFromAlliance(DriverStation.getAlliance());
                    registerPathEvents(shooter, pivotWheels, intake, hopper);
                }),
                Commands.defer(
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
                        Set.of(swerveSubsystem)),
                createTagAlignCommand(swerveSubsystem, aprilTag),
                // Final stop (in case events didn't stop shooter/hopper)
                Commands.runOnce(
                        () -> {
                            shooter.stop();
                            hopper.stop();
                        },
                        shooter,
                        hopper),
                Commands.runOnce(swerveSubsystem::stop, swerveSubsystem));
    }

    private static void registerPathEvents(
            Shooter shooter,
            frc.robot.subsystems.intake.PivotWheels pivotWheels,
            frc.robot.subsystems.intake.intake intake,
            frc.robot.subsystems.intake.hopper.hopper hopper) {
        // Marker name: "intake"
        NamedCommands.registerCommand(
                "intake",
                Commands.runOnce(
                        () -> {
                            intake.moveToIntakePosition();
                            pivotWheels.intakeIn();
                            hopper.intakeIn();
                        },
                        intake,
                        pivotWheels,
                        hopper));

        // Marker name: "deintake"
        NamedCommands.registerCommand(
                "deintake",
                Commands.runOnce(
                        () -> {
                            pivotWheels.stop();
                            hopper.stop();
                            intake.moveToStowedPosition();
                        },
                        pivotWheels,
                        hopper,
                        intake));

        // Marker name: "shooter"
        NamedCommands.registerCommand(
                "shooter",
                Commands.sequence(
                        Commands.runOnce(() -> shooter.spinAll(0.5), shooter),
                        Commands.runOnce(hopper::intakeIn, hopper),
                        Commands.waitSeconds(2.0),
                        Commands.runOnce(
                                () -> {
                                    shooter.stop();
                                    hopper.stop();
                                },
                                shooter,
                                hopper)));
    }

    public static Command createTagAlignCommand(SwerveSubsystem swerveSubsystem, AprilTag aprilTag) {
        // Delegate to teleop Alignment helper
        return frc.robot.teleop.Alignment.createTagAlignCommand(swerveSubsystem, aprilTag, TARGET_TAG_ID);
    }

    /** Create an align-to-tag command for an arbitrary tag ID. */
    public static Command createTagAlignCommand(SwerveSubsystem swerveSubsystem, AprilTag aprilTag, int tagId) {
        // Delegate to teleop Alignment helpers so alignment logic is centralized
        return frc.robot.teleop.Alignment.createTagAlignCommand(swerveSubsystem, aprilTag, tagId);
    }

    public static Command createScanAndRunTagCommand(SwerveSubsystem swerveSubsystem, AprilTag aprilTag) {
        final int[] detectedTagId = {-1};

        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            detectedTagId[0] = -1;
                            aprilTag.updateOriginFromAlliance(DriverStation.getAlliance());
                        }),
                Commands.run(
                                () -> aprilTag.getBestVisibleTarget()
                                        .ifPresent(
                                                target -> {
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

    // Alignment helpers moved to frc.robot.teleop.Alignment

    private static boolean atHeading(Rotation2d current, Rotation2d target) {
        return Math.abs(headingErrorRad(current, target)) <= Math.toRadians(HEADING_TOLERANCE_DEG);
    }

    private static double headingErrorRad(Rotation2d current, Rotation2d target) {
        return MathUtil.angleModulus(target.getRadians() - current.getRadians());
    }
}
