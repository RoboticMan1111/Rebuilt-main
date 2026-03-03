package frc.robot.subsystems.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.AprilTag;
import frc.robot.subsystems.intake.PivotWheels;
import frc.robot.subsystems.intake.intake;
import frc.robot.subsystems.intake.hopper.hopper;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Red-side variant of the simple middle auto: drive backwards ~5ft (1.524 m) then shoot.
 * Keeping behavior explicit in its own class makes it easy to diverge later.
 */
public class RedMiddleAuto {
    private static final double BACKUP_METERS = 1.524; // 5 feet
    private static final double DRIVE_SPEED_MPS = 0.6; // speed for backward motion

    private RedMiddleAuto() {}

    public static Command build(SwerveSubsystem swerveSubsystem, AprilTag aprilTag, Shooter shooter,
            PivotWheels pivotWheels, intake intakePivot, hopper hopper) {

        final Pose2d[] startPose = new Pose2d[1];

        Command driveBack = Commands.sequence(
                Commands.runOnce(() -> startPose[0] = swerveSubsystem.getPose(), swerveSubsystem),
                Commands.run(() -> {
                    swerveSubsystem.driveRobotRelative(new ChassisSpeeds(-DRIVE_SPEED_MPS, 0.0, 0.0));
                }, swerveSubsystem)
                        .until(() -> {
                            Pose2d current = swerveSubsystem.getPose();
                            double dx = current.getTranslation().getX() - startPose[0].getTranslation().getX();
                            double dy = current.getTranslation().getY() - startPose[0].getTranslation().getY();
                            double distance = Math.hypot(dx, dy);
                            return distance >= BACKUP_METERS;
                        })
                        .withTimeout(4.0)
                        .andThen(Commands.runOnce(swerveSubsystem::stop, swerveSubsystem)));

        Command shootSequence = Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.spinAll(0.5);
                    hopper.intakeIn();
                }, shooter, hopper),
                Commands.waitSeconds(2.0),
                Commands.runOnce(() -> {
                    shooter.stop();
                    hopper.stop();
                }, shooter, hopper));

    return Commands.sequence(
        Commands.runOnce(() -> aprilTag.updateOriginFromAlliance(DriverStation.getAlliance())),
        driveBack,
        shootSequence,
        Commands.runOnce(swerveSubsystem::stop, swerveSubsystem));
    }
}
