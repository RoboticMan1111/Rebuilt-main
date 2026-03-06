package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  private static final int PIVOT_MOTOR_ID = 13;
  private static final double PIVOT_GEAR_RATIO = 45.0; // motor rotations : arm rotations
  private static final double PIVOT_DEADBAND = 0.10;
  private static final double MAX_MANUAL_OUTPUT = 0.30;
  private static final double MAX_PIVOT_OUTPUT_DOWN = 0.18;
  private static final double MAX_PIVOT_OUTPUT_UP = 0.30;
  // If direction is opposite on robot, flip this to false.
  private static final boolean POSITIVE_OUTPUT_MOVES_DOWN = true;
  private static final double PIVOT_KP = 0.08;
  private static final double PIVOT_KI = 0.0;
  private static final double PIVOT_KD = 0.0;
  private static final double PIVOT_TOLERANCE_ROT = 0.75 / PIVOT_GEAR_RATIO;
  // Minimum (most-retracted / rearward) safe pivot rotation. Tune on robot.
  // Negative means rotated rearward from the zero reference.
  private static final double MIN_PIVOT_ROT = -0.50; // ~ -180deg (adjust after testing)
  private static final double MAX_PIVOT_ROT = 1.50; // safe forward limit (adjust after testing)
  private static final double STOWED_POSITION_ROT = MIN_PIVOT_ROT;
  private static final double INTAKE_POSITION_ROT = 18.0 / PIVOT_GEAR_RATIO;

  private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private final PIDController pivotController = new PIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD);

  private boolean positionControlEnabled = true;
  private double targetPositionRot = STOWED_POSITION_ROT;

  @SuppressWarnings("deprecation")
  public intake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(25);
    config.inverted(false);
    // Expose arm rotations in software instead of raw motor rotations.
    config.encoder.positionConversionFactor(1.0 / PIVOT_GEAR_RATIO);
    config.encoder.velocityConversionFactor(1.0 / PIVOT_GEAR_RATIO);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // Clamp initial encoder position to a safe range in case the encoder value
    // or robot wiring yields an out-of-range value at boot.
    double initial = MathUtil.clamp(STOWED_POSITION_ROT, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    pivotEncoder.setPosition(initial);
    pivotController.setTolerance(PIVOT_TOLERANCE_ROT);
  }

  public void setPivotFromJoystick(double rightY) {
    double output = MathUtil.applyDeadband(rightY, PIVOT_DEADBAND) * MAX_MANUAL_OUTPUT;
    if (Math.abs(output) > 0.0) {
      positionControlEnabled = false;
      pivotMotor.set(clampPivotOutput(output));
      return;
    }

    if (!positionControlEnabled) {
      // When leaving manual control, capture current position as the new
      // target (clamped) so the PID can hold the arm and prevent falling.
      setTargetPositionRot(getPivotPositionRot());
    }
  }

  public void moveToIntakePosition() {
    setTargetPositionRot(INTAKE_POSITION_ROT);
  }

  public void moveToStowedPosition() {
    setTargetPositionRot(STOWED_POSITION_ROT);
  }

  public void setTargetPositionRot(double targetRot) {
    // keep targets inside the safe travel range
    targetPositionRot = MathUtil.clamp(targetRot, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    positionControlEnabled = true;
  }

  public double getPivotPositionRot() {
    return pivotEncoder.getPosition();
  }

  public boolean atTargetPosition() {
    return Math.abs(targetPositionRot - getPivotPositionRot()) <= PIVOT_TOLERANCE_ROT;
  }

  @Override
  public void periodic() {
    if (positionControlEnabled) {
      double output = pivotController.calculate(getPivotPositionRot(), targetPositionRot);
      pivotMotor.set(clampPivotOutput(output));
    }

  }

  public void stop() {
    positionControlEnabled = false;
    pivotMotor.stopMotor();
  }

  private double clampPivotOutput(double requestedOutput) {
    double maxDown = POSITIVE_OUTPUT_MOVES_DOWN ? MAX_PIVOT_OUTPUT_DOWN : MAX_PIVOT_OUTPUT_UP;
    double maxUp = POSITIVE_OUTPUT_MOVES_DOWN ? MAX_PIVOT_OUTPUT_UP : MAX_PIVOT_OUTPUT_DOWN;
    return MathUtil.clamp(requestedOutput, -maxUp, maxDown);
  }
}
