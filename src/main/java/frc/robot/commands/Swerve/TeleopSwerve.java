package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Simple teleop command for swerve using CommandXboxController.
 * Left stick -> translation (forward/back + strafe)
 * Right stick X -> rotation
 */
public class TeleopSwerve extends Command {
  private final SwerveSubsystem swerve;
  private final CommandXboxController controller;
  private final SlewRateLimiter forwardLimiter =
      new SlewRateLimiter(OIConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter strafeLimiter =
      new SlewRateLimiter(OIConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(OIConstants.ROTATION_SLEW_RATE);

  public TeleopSwerve(SwerveSubsystem swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    forwardLimiter.reset(0.0);
    strafeLimiter.reset(0.0);
    rotationLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    double forward = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.DEADBAND);
    double strafe = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.DEADBAND);
    double rotation = -MathUtil.applyDeadband(controller.getRightX(), OIConstants.DEADBAND);
    forward = Math.copySign(Math.pow(Math.abs(forward), OIConstants.TRANSLATION_EXPO), forward);
    strafe = Math.copySign(Math.pow(Math.abs(strafe), OIConstants.TRANSLATION_EXPO), strafe);
    rotation = Math.copySign(Math.pow(Math.abs(rotation), OIConstants.ROTATION_EXPO), rotation);
    forward *= OIConstants.TELEOP_TRANSLATION_SCALE;
    strafe *= OIConstants.TELEOP_TRANSLATION_SCALE;
    rotation *= OIConstants.TELEOP_ROTATION_SCALE;

    forward = forwardLimiter.calculate(forward);
    strafe = strafeLimiter.calculate(strafe);
    rotation = rotationLimiter.calculate(rotation);

    SmartDashboard.putNumber("Swerve/Diag/DriverForwardCmd", forward);
    SmartDashboard.putNumber("Swerve/Diag/DriverStrafeCmd", strafe);
    SmartDashboard.putNumber("Swerve/Diag/DriverRotateCmd", rotation);
    SmartDashboard.putBoolean(
        "Swerve/Diag/DriverInputActive",
        Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotation) > 0.05);

    // Robot-relative control during debug: removes gyro/heading transform as a drive blocker.
    swerve.drive(forward, strafe, rotation, false, false, false);
  }
}
