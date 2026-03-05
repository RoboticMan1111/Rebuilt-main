package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.AprilTag;
import frc.robot.subsystems.intake.PivotWheels;
import frc.robot.subsystems.intake.hopper.hopper;
import frc.robot.subsystems.intake.intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Set;

public class RedLeftAuto {
    private static final String PATH_FILE = "RedLeftAuto";
    // Starting test speed for shooter in deintake marker.
    private static final double DEINTAKE_SHOOT_SPEED = 0.55;

    private RedLeftAuto() {}

    public static Command build(
            SwerveSubsystem swerveSubsystem,
            AprilTag aprilTag,
            Shooter shooter,
            PivotWheels pivotWheels,
            intake intakePivot,
            hopper hopperSubsystem) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    aprilTag.updateOriginFromAlliance(DriverStation.getAlliance());
                    registerPathEvents(shooter, pivotWheels, intakePivot, hopperSubsystem);
                }),
                Commands.defer(
                        () -> {
                            try {
                                PathPlannerPath path = PathPlannerPath.fromPathFile(PATH_FILE);
                                return AutoBuilder.followPath(path);
                            } catch (Exception ex) {
                                DriverStation.reportWarning(
                                        "RedLeftAuto path load failed for '" + PATH_FILE + "': " + ex.getMessage(),
                                        false);
                                return Commands.none();
                            }
                        },
                        Set.of(swerveSubsystem)),
                Commands.runOnce(
                        () -> {
                            shooter.stop();
                            hopperSubsystem.stop();
                            pivotWheels.stop();
                        },
                        shooter,
                        hopperSubsystem,
                        pivotWheels),
                Commands.runOnce(swerveSubsystem::stop, swerveSubsystem));
    }

    private static void registerPathEvents(
            Shooter shooter,
            PivotWheels pivotWheels,
            intake intakePivot,
            hopper hopperSubsystem) {
        // Marker name: "intake"
        NamedCommands.registerCommand(
                "intake",
                Commands.runOnce(
                        () -> {
                            intakePivot.moveToIntakePosition();
                            pivotWheels.intakeIn();
                            // Feed while intaking so notes move smoothly toward shooter.
                            hopperSubsystem.intakeIn();
                        },
                        intakePivot,
                        pivotWheels,
                        hopperSubsystem));

        // Marker name: "deintake"
        NamedCommands.registerCommand(
                "deintake",
                Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    pivotWheels.stop();
                                    intakePivot.moveToStowedPosition();
                                    shooter.spinAll(DEINTAKE_SHOOT_SPEED);
                                    hopperSubsystem.intakeIn();
                                },
                                pivotWheels,
                                intakePivot,
                                shooter,
                                hopperSubsystem),
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                                () -> {
                                    shooter.stop();
                                    hopperSubsystem.stop();
                                },
                                shooter,
                                hopperSubsystem)));

        // Marker name: "shooter"
        NamedCommands.registerCommand(
                "shooter",
                Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    shooter.spinAll(DEINTAKE_SHOOT_SPEED);
                                    hopperSubsystem.intakeIn();
                                },
                                shooter,
                                hopperSubsystem),
                        Commands.waitSeconds(2.0),
                        Commands.runOnce(
                                () -> {
                                    shooter.stop();
                                    hopperSubsystem.stop();
                                },
                                shooter,
                                hopperSubsystem)));
    }
}
