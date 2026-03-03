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
 * Simple middle auto: drive backwards ~5ft (1.524 m) then shoot.
 * This is provided for both blue/red middle selections. The command
 * sequence updates the AprilTag origin for alliance before running.
 */
public class BlueMiddleAuto {
	private static final double BACKUP_METERS = 1.524; // 5 feet
	private static final double DRIVE_SPEED_MPS = 0.6; // forward speed magnitude (we'll use negative vx to go backward)

	private BlueMiddleAuto() {}

	/**
	 * Build the middle auto. Signature mirrors other autos so RobotContainer can pass the same subsystems.
	 */
	public static Command build(SwerveSubsystem swerveSubsystem, AprilTag aprilTag, Shooter shooter,
			PivotWheels pivotWheels, intake intakePivot, hopper hopper) {

		// Command that records the starting pose, then drives backwards until we have moved ~BACKUP_METERS
		final Pose2d[] startPose = new Pose2d[1];

		Command driveBack = Commands.sequence(
				Commands.runOnce(() -> startPose[0] = swerveSubsystem.getPose(), swerveSubsystem),
				Commands.run(() -> {
					// Drive robot-relative backward by setting negative vx
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

		// Simple shooting sequence: spin shooter and run hopper to feed for 2 seconds
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
				// Update AprilTag field origin for alliance (keeps behavior consistent with other autos)
				Commands.runOnce(() -> aprilTag.updateOriginFromAlliance(DriverStation.getAlliance())),
				driveBack,
				shootSequence,
				// ensure chassis is stopped at the end
				Commands.runOnce(swerveSubsystem::stop, swerveSubsystem));
	}
}