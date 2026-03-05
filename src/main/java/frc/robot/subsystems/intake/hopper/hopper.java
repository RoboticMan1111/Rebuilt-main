package frc.robot.subsystems.intake.hopper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hopper extends SubsystemBase {
  private static final int HOPPER_MOTOR_ID = 10;
  private static final int HOPPER_AUX_NEO_ID = 25;
  private static final double MAX_OUTPUT = 0.7;
  private static final double TRIGGER_DEADBAND = 0.05;
  // Ratios are motor:hopper-shaft.
  private static final double PRIMARY_GEAR_RATIO = 1.0;
  private static final double AUX_GEAR_RATIO = 3.0;
  // Set to -1.0 so positive feed command spins primary hopper motor counterclockwise.
  private static final double PRIMARY_FEED_DIRECTION = -1.0;

  private final SparkMax hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax hopperAuxNeo = new SparkMax(HOPPER_AUX_NEO_ID, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public hopper() {
  SparkMaxConfig config = new SparkMaxConfig();
  config.idleMode(IdleMode.kBrake);
  config.smartCurrentLimit(40);
  config.inverted(false);
  // Neo vortex is 1:1 gear ratio on the hopper so expose wheel rotations directly
  // (motor rotations = wheel rotations). This sets encoder conversions to 1:1.
  config.encoder.positionConversionFactor(1.0);
  config.encoder.velocityConversionFactor(1.0);
  hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  SparkMaxConfig auxConfig = new SparkMaxConfig();
  auxConfig.idleMode(IdleMode.kBrake);
  auxConfig.smartCurrentLimit(40);
  auxConfig.inverted(false);
  // NEO on id 25 has 3:1 motor:shaft reduction.
  auxConfig.encoder.positionConversionFactor(1.0 / AUX_GEAR_RATIO);
  auxConfig.encoder.velocityConversionFactor(1.0 / AUX_GEAR_RATIO);
  hopperAuxNeo.configure(auxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeIn() {
    setBothFromCommand(MAX_OUTPUT);
  }

  public void setFromTriggers(double leftTrigger, double rightTrigger) {
    double left = MathUtil.applyDeadband(leftTrigger, TRIGGER_DEADBAND);
    double right = MathUtil.applyDeadband(rightTrigger, TRIGGER_DEADBAND);
    double output = (right - left) * MAX_OUTPUT;
    setBothFromCommand(output);
  }

  public void stop() {
    hopperMotor.stopMotor();
    hopperAuxNeo.stopMotor();
  }

  // Convert the shared hopper command into per-motor output with gear-ratio compensation.
  private void setBothFromCommand(double command) {
    double clamped = MathUtil.clamp(command, -1.0, 1.0);
    double primaryScale = PRIMARY_GEAR_RATIO;
    double auxScale = AUX_GEAR_RATIO;
    double norm = Math.max(primaryScale, auxScale);

    double primaryOut = MathUtil.clamp(clamped * (primaryScale / norm), -1.0, 1.0);
    double auxOut = MathUtil.clamp(clamped * (auxScale / norm), -1.0, 1.0);

    hopperMotor.set(primaryOut * PRIMARY_FEED_DIRECTION);
    hopperAuxNeo.set(auxOut);
  }
}
