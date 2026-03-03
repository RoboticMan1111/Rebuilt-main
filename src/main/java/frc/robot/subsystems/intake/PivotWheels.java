package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotWheels extends SubsystemBase {
  // TODO: Set this to the real CAN ID for the NEO 550 that drives the small grey wheels.
  private static final int PIVOT_WHEELS_MOTOR_ID = 52;
  private static final double INTAKE_OUTPUT = 0.8;

  private final SparkMax pivotWheelsMotor =
      new SparkMax(PIVOT_WHEELS_MOTOR_ID, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public PivotWheels() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);
    config.inverted(false);
    pivotWheelsMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeIn() {
    pivotWheelsMotor.set(MathUtil.clamp(INTAKE_OUTPUT, -1.0, 1.0));
  }

  /** Set pivot wheels speed directly ([-1,1]). Right trigger -> positive. */
  public void set(double speed) {
    pivotWheelsMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stop() {
    pivotWheelsMotor.stopMotor();
  }
}
