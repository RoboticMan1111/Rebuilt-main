package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Gyro wrapper that optionally applies an installation offset.
 *
 * Important: the default offset is 0.0 degrees. A historical hard-coded
 * 45° offset caused unexpected heading jumps when other code also
 * applied transforms. Use the Preferences key "Gyro/initialOffsetDegrees"
 * only when you intentionally need a static installation offset.
 */
public class Gyro {
    private final Pigeon2 pigeon;
    private final double initialOffsetDegrees;

    public Gyro() {
        pigeon = new Pigeon2(0);
        // Load runtime-configurable initial offset (default 0.0). This avoids
        // surprising 45° jumps from a hard-coded constant.
        initialOffsetDegrees = Preferences.getDouble("Gyro/initialOffsetDegrees", 0.0);
        SmartDashboard.putNumber("Gyro/initialOffsetDegrees", initialOffsetDegrees);
    }

    public Rotation2d getRotation2d() {
        double yawDegrees = pigeon.getYaw().getValueAsDouble() + initialOffsetDegrees;
        return Rotation2d.fromDegrees(yawDegrees);
    }

    public void resetGyro(double degrees) {
        // Compensate for read-time offset so callers can set true heading directly.
        pigeon.setYaw(degrees - initialOffsetDegrees);
    }
}
