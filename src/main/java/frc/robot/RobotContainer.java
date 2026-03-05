// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.AprilTag;
import frc.robot.subsystems.intake.PivotWheels;
import frc.robot.subsystems.intake.Wheel;
import frc.robot.subsystems.auto.BlueLeftAuto;
import frc.robot.subsystems.auto.BlueMiddleAuto;
import frc.robot.subsystems.auto.RedLeftAuto;
import frc.robot.subsystems.auto.RedMiddleAuto;
import frc.robot.subsystems.intake.hopper.hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.intake;


/**
 * Minimal RobotContainer focused on teleop swerve mapping.
 * Left stick = translation (forward/back + strafe), right X = rotation.
 */
public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final AprilTag m_aprilTag = new AprilTag();
  private final hopper m_hopper = new hopper();
  private final Shooter m_shooter = new Shooter();
  private final PivotWheels m_pivotWheels = new PivotWheels();
  private final Wheel m_wheel = new Wheel();
  private final intake m_intakePivot = new intake();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    SmartDashboard.putData("Auto chooser", autoChooser);

    // Default teleop swerve mapping: left stick translation, right stick X rotation.
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, m_driverController));

    // Operator controls intake pivot with right Y axis.
    m_intakePivot.setDefaultCommand(
        Commands.run(
            () -> m_intakePivot.setPivotFromJoystick(m_operatorController.getRightY()), m_intakePivot));

    // Operator controls hopper with triggers.
    m_hopper.setDefaultCommand(
        Commands.run(
            () ->
                m_hopper.setFromTriggers(
                    m_operatorController.getLeftTriggerAxis(), m_operatorController.getRightTriggerAxis()),
            m_hopper));

  // Driver controls pivot wheels & blue wheels with triggers (right = intake in, left = reverse)
  m_pivotWheels.setDefaultCommand(
    Commands.run(
      () -> {
        double out = m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis();
        m_pivotWheels.set(out);
        // Blue wheels: Wheel.setFromTriggers expects (left,right)
        m_wheel.setFromTriggers(m_driverController.getLeftTriggerAxis(), m_driverController.getRightTriggerAxis());
      },
      m_pivotWheels,
      m_wheel));

  autoChooser.setDefaultOption("BlueLeftAuto", BlueLeftAuto.build(m_swerveSubsystem, m_aprilTag, m_shooter, m_pivotWheels, m_intakePivot, m_hopper));
  // Provide other (placeholder/fallback) options so the driver station chooser
  // has multiple selections. These currently reuse BlueLeftAuto implementation
  // as a safe fallback until specific auto implementations are added.
  autoChooser.addOption("BlueMiddleAuto", BlueMiddleAuto.build(m_swerveSubsystem, m_aprilTag, m_shooter, m_pivotWheels, m_intakePivot, m_hopper));
  autoChooser.addOption("BlueRightAuto", BlueLeftAuto.build(m_swerveSubsystem, m_aprilTag, m_shooter, m_pivotWheels, m_intakePivot, m_hopper));
  autoChooser.addOption("RedLeftAuto", RedLeftAuto.build(m_swerveSubsystem, m_aprilTag, m_shooter, m_pivotWheels, m_intakePivot, m_hopper));
  autoChooser.addOption("RedMiddleAuto", RedMiddleAuto.build(m_swerveSubsystem, m_aprilTag, m_shooter, m_pivotWheels, m_intakePivot, m_hopper));

    configureBindings();
  }

  private void configureBindings() {
    // Hold X on driver controller for intake preset:
    // lower intake bar + run pivot wheels + run blue intake wheels.
    // Allow the operator to always control the intake pivot with the
    // right joystick Y axis. To make the preset usable but non-blocking,
    // the preset command will NOT require the intake pivot subsystem.
    // That way the operator's default command (which reads the joystick)
    // can run and override the preset while it's active.
    m_driverController
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_intakePivot.moveToIntakePosition();
                  m_pivotWheels.intakeIn();
                  m_hopper.intakeIn();
                },
                () -> {
                  m_pivotWheels.stop();
                  m_hopper.stop();
                  m_intakePivot.moveToStowedPosition();
                },
                /* Intentionally do NOT require m_intakePivot so the operator's
                   joystick control can always run and override the preset. */
                m_pivotWheels,
                m_hopper));

    // Press B: reset gyro heading to 0 degrees.
    m_driverController.b().onTrue(Commands.runOnce(() -> m_swerveSubsystem.resetGyro(0.0), m_swerveSubsystem));

    // Hold A on operator controller to run shooter wheels and hopper while held.
    m_operatorController
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_shooter.spinAll();
                  m_hopper.intakeIn();
                },
                () -> {
                  m_shooter.stop();
                  m_hopper.stop();
                },
                m_shooter,
                m_hopper));

  // Press Y to scan visible AprilTag IDs and run a context-aware action:
  // - If tag 23 is seen -> rotate to 45deg and shoot
  // - If tag 25 or 26 (hub tags) -> run the tag-align-to-hub routine and then shoot
  m_driverController.y().onTrue(createFlexibleScanAndRunTagCommand());

  }

  private Command createFlexibleScanAndRunTagCommand() {
    final int[] detectedId = {-1};
    final double shootHeadingDeg = 45.0;
    final double headingKp = 2.4;
    final double maxOmegaRadPerSec = 2.0;
    final double headingToleranceDeg = 2.0;

    // Command to rotate to a fixed heading (used for tag 23)
    Command rotateToShootHeading = Commands.run(
            () -> {
              double currentHeadingRad = m_swerveSubsystem.getPose().getRotation().getRadians();
              double targetHeadingRad = Math.toRadians(shootHeadingDeg);
              double errorRad = MathUtil.angleModulus(targetHeadingRad - currentHeadingRad);
              double omega = MathUtil.clamp(errorRad * headingKp, -maxOmegaRadPerSec, maxOmegaRadPerSec);
              m_swerveSubsystem.driveFieldRelative(new ChassisSpeeds(0.0, 0.0, omega));
            },
            m_swerveSubsystem)
        .until(() -> {
          double currentHeadingRad = m_swerveSubsystem.getPose().getRotation().getRadians();
          double targetHeadingRad = Math.toRadians(shootHeadingDeg);
          return Math.abs(MathUtil.angleModulus(targetHeadingRad - currentHeadingRad))
              <= Math.toRadians(headingToleranceDeg);
        })
        .withTimeout(1.5)
        .andThen(Commands.runOnce(m_swerveSubsystem::stop, m_swerveSubsystem));

    // Command sequence to shoot for a short duration.
    // Feed path uses hopper.intakeIn(), which drives both hopper motors (including CAN ID 25).
    Command shootSequence = Commands.sequence(
        Commands.runOnce(() -> {
          m_shooter.spinAll(0.5);
          m_hopper.intakeIn();
        }, m_shooter, m_hopper),
        Commands.waitSeconds(2.0),
        Commands.runOnce(() -> {
          m_shooter.stop();
          m_hopper.stop();
        }, m_shooter, m_hopper));

    

    // Main sequence: detect any visible tag, then branch based on ID
    Command pollForTag = Commands.run(
            () -> m_aprilTag.getBestVisibleTarget().ifPresent(t -> detectedId[0] = t.getFiducialId()))
        .until(() -> detectedId[0] != -1)
        .withTimeout(1.5);

    // Defer branch selection until after detection so we can build a command
    // that uses the detected tag id at runtime.
    Command branch = Commands.defer(() -> {
          int id = detectedId[0];
          if (id == 23) {
            return Commands.sequence(rotateToShootHeading, shootSequence);
          }
          // For hub tags on either alliance, align to the detected tag then shoot.
          if (id == 25 || id == 26 || id == 9 || id == 10) {
            return Commands.sequence(BlueLeftAuto.createTagAlignCommand(m_swerveSubsystem, m_aprilTag, id), shootSequence);
          }
          return Commands.runOnce(() -> SmartDashboard.putString("Vision/ScanResult", "No actionable tag"));
        }, Set.of(m_swerveSubsystem, m_shooter, m_hopper));

    return Commands.sequence(
        Commands.runOnce(() -> {
          detectedId[0] = -1;
          m_aprilTag.updateOriginFromAlliance(DriverStation.getAlliance());
        }),
        pollForTag,
        branch);
  }

  /** Returns selected autonomous command or null. */
  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    if (selected != null) {
      return selected;
    }
  DriverStation.reportWarning("Auto chooser returned null, falling back to BlueLeftAuto.", false);
  return BlueLeftAuto.build(m_swerveSubsystem, m_aprilTag, m_shooter, m_pivotWheels, m_intakePivot, m_hopper);
  }
}
