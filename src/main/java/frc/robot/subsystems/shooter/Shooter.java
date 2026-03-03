package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import java.lang.reflect.Method;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple shooter subsystem with 4 motors:
 * - Two Krakens (implemented here with SparkFlex) with CAN IDs 5 and 6
 * - Two NEOs (Spark Max) with CAN IDs 7 and 8
 *
 * Pressing operator A will spin all shooter wheels (wiring is done in RobotContainer).
 */
public class Shooter extends SubsystemBase {
  // IDs: Krakens use TalonFX controllers with built-in encoders
  private static final int KRAKEN_1_ID = 5;
  private static final int KRAKEN_2_ID = 6;
  private static final int NEO_1_ID = 7;
  private static final int NEO_2_ID = 9; // moved to 9 to avoid conflict with hopper

  private static final double DEFAULT_SPEED = 1.0;

  private final SparkMax neo1 = new SparkMax(NEO_1_ID, MotorType.kBrushless);
  private final SparkMax neo2 = new SparkMax(NEO_2_ID, MotorType.kBrushless);

  // Krakens: use TalonFX (Phoenix 6) controllers which provide encoder reads
  private final TalonFX kraken1 = new TalonFX(KRAKEN_1_ID);
  private final TalonFX kraken2 = new TalonFX(KRAKEN_2_ID);

  // Simple follower mode: when enabled the shooter will read the encoder
  // velocities and command the motors to follow those values (scaled).
  private boolean encoderFollowerEnabled = false;
  private static final double MAX_ENCODER_VELOCITY = 6000.0; // tune per hardware

  public Shooter() {
    // Note: Kraken controllers are not configured here because they are not
    // represented by SparkFlex/SparkMax classes. If your Kraken vendor API
    // requires configuration, perform it here (or add a Kraken-specific
    // config wrapper). For the NEOs we still configure via SparkMaxConfig.
    SparkMaxConfig nConfig = new SparkMaxConfig();
    nConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    nConfig.smartCurrentLimit(40);
    nConfig.inverted(false);
    neo1.configure(nConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
    neo2.configure(nConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // If encoder follower mode is enabled, read encoder velocities from
    // the Kraken controllers and command the motors to follow that
    // velocity (scaled to [-1,1]). This lets an external encoder
    // measurement drive the shooter outputs.
    if (encoderFollowerEnabled) {
  // Read velocities from TalonFX built-in sensors. Phoenix6 uses the
  // getRotorVelocity() accessor which returns a Measurement object.
  double v1 = kraken1.getRotorVelocity().getValueAsDouble();
  double v2 = kraken2.getRotorVelocity().getValueAsDouble();
      double s1 = Math.max(-1.0, Math.min(1.0, v1 / MAX_ENCODER_VELOCITY));
      double s2 = Math.max(-1.0, Math.min(1.0, v2 / MAX_ENCODER_VELOCITY));
      // Command TalonFX percent output via reflection (handles different Phoenix6 method names)
      talonSetPercent(kraken1, s1);
      talonSetPercent(kraken2, s2);
      // Mirror to NEOs so all shooter wheels spin together
      neo1.set((s1 + s2) / 2.0);
      neo2.set((s1 + s2) / 2.0);
    }
  }

  /** Enable/disable encoder follower mode. */
  public void setEncoderFollowerEnabled(boolean enabled) {
    encoderFollowerEnabled = enabled;
  }

  /** Read integrated Kraken encoder position (rotations). */
  public double getKraken1Position() {
    return kraken1.getRotorPosition().getValueAsDouble();
  }

  /** Read integrated Kraken encoder position (rotations). */
  public double getKraken2Position() {
    return kraken2.getRotorPosition().getValueAsDouble();
  }

  /** Read integrated Kraken encoder velocity (RPM or controller units). */
  public double getKraken1Velocity() {
    return kraken1.getRotorVelocity().getValueAsDouble();
  }

  /** Read integrated Kraken encoder velocity (RPM or controller units). */
  public double getKraken2Velocity() {
    return kraken2.getRotorVelocity().getValueAsDouble();
  }

  /** Spin all shooter wheels at default speed. */
  public void spinAll() {
    spinAll(DEFAULT_SPEED);
  }

  /** Spin all shooter wheels at the given speed ([-1,1]). */
  public void spinAll(double speed) {
    double s = Math.max(-1.0, Math.min(1.0, speed));
    talonSetPercent(kraken1, s);
    talonSetPercent(kraken2, s);
    neo1.set(s);
    neo2.set(s);
  }

  /** Stop all shooter motors. */
  public void stop() {
    // Stop TalonFX outputs
    talonSetPercent(kraken1, 0.0);
    talonSetPercent(kraken2, 0.0);
    neo1.stopMotor();
    neo2.stopMotor();
  }

  /** Try to set a TalonFX percent-output using reflection to accommodate Phoenix6 API differences. */
  private void talonSetPercent(TalonFX talon, double output) {
    try {
      // Try common method names first
      Method m = null;
      try {
        m = talon.getClass().getMethod("setPercentOutput", double.class);
      } catch (NoSuchMethodException ignored) {
      }
      if (m == null) {
        try {
          m = talon.getClass().getMethod("set", double.class);
        } catch (NoSuchMethodException ignored) {
        }
      }
      // Last resort: find any single-double setter method whose name contains "percent" or "set"
      if (m == null) {
        for (Method cand : talon.getClass().getMethods()) {
          Class<?>[] params = cand.getParameterTypes();
          if (params.length == 1 && params[0] == double.class) {
            String name = cand.getName().toLowerCase();
            if (name.contains("percent") || name.equals("set")) {
              m = cand;
              break;
            }
          }
        }
      }
      if (m != null) {
        m.invoke(talon, output);
      }
    } catch (Exception e) {
      // Reflection failure - avoid crashing robot code. In deployment, replace with direct TalonFX API calls.
      // Optionally log once to SmartDashboard (omitted here to keep periodic fast).
    }
  }
  
}
