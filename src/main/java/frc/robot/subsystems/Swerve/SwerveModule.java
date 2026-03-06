package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private static final double DRIVE_SUPPRESS_ANGLE_DEG = 45.0;
    private static final double TURN_DEADBAND_DEG = 2.0;
    private static final double DRIVE_REQUEST_DEADBAND_MPS = 0.18;

    private SparkMax driveMotor;
    private SparkMax turnMotor;
    private CoreCANcoder absoluteEncoder;
    private final double absoluteOffsetRad;
    private final int absoluteEncoderId;

    private RelativeEncoder driveEncoder;
    private double lastDriveCommand = 0.0;
    private double lastTurnCommand = 0.0;
    private Rotation2d lastValidAngle = new Rotation2d();
    private boolean hasValidAngle = false;


    public SwerveModule(int driveMotorPort, int turningMotorPort, int absoluteEncoderPort, boolean driveInverted, boolean turnInverted, double angleOffsetRad) {
        driveMotor = createMotorController(driveMotorPort, driveInverted, true);
        turnMotor = createMotorController(turningMotorPort, turnInverted, false);
        absoluteEncoder = new CoreCANcoder(absoluteEncoderPort);
        absoluteEncoderId = absoluteEncoderPort;
        absoluteOffsetRad = angleOffsetRad;

        driveEncoder = createEncoder(driveMotor);
    }

    /**
     * Optimize a target state so the wheel rotates the minimal amount from the current angle.
     * If the angle difference is greater than 90 degrees, flip the wheel speed and add 180deg to the angle.
     */
    private SwerveModuleState optimizeStateForMinimalRotation(SwerveModuleState targetState, Rotation2d currentAngle) {
        Rotation2d desired = targetState.angle;
        double delta = MathUtil.angleModulus(desired.getRadians() - currentAngle.getRadians());
        if (Math.abs(delta) > Math.PI / 2.0) {
            // Reverse wheel direction and rotate desired angle by pi
            double flippedSpeed = -targetState.speedMetersPerSecond;
            Rotation2d newAngle = desired.rotateBy(Rotation2d.fromRadians(Math.PI));
            return new SwerveModuleState(flippedSpeed, newAngle);
        }
        return targetState;
    }

    @SuppressWarnings("deprecation")
    private SparkMax createMotorController(int port, boolean isInverted, boolean isDriveMotor) {
        SparkMax controller = new SparkMax(port, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.voltageCompensation(SwerveConstants.VOLTAGE_COMPENSATION);
        config.idleMode(IdleMode.kBrake);
        config.openLoopRampRate(SwerveConstants.RAMP_RATE);
        config.closedLoopRampRate(SwerveConstants.RAMP_RATE); 

        config.smartCurrentLimit(SwerveConstants.CURRENT_LIMIT);

        config.inverted(isInverted);

        // Drive encoder conversion factors are wheel-distance conversions and only apply to the drive motor.
        // The turn motor uses the CANcoder (absolute encoder) for angle, not its relative encoder.
        if (isDriveMotor) {
            config.encoder.velocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);
            config.encoder.positionConversionFactor(SwerveConstants.POSITION_CONVERSION_FACTOR);
        }
        // for some reason causes robot to shake:
        //     controller.burnFlash(); 
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return controller;
    }

    private RelativeEncoder createEncoder(SparkMax controller) {
        RelativeEncoder encoder = controller.getEncoder();
        // convert from native unit of rpm to m/s
        //encoder.setVelocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);

        return encoder;
    }

    public void setTargetState(SwerveModuleState targetState, PIDController drivePID, ProfiledPIDController turnPID) {
        // get angle + optimize angle 
        Rotation2d currentAngle = getAngle();
        boolean canCoderOnline = isCanCoderOnline();
        // Optimize the desired state relative to the current angle. The built-in SwerveModuleState.optimize
        // is deprecated, so perform an equivalent optimization here to avoid the deprecation warning.
        SwerveModuleState optimizedState = optimizeStateForMinimalRotation(targetState, currentAngle);

        // calculate the motor output + set the motor states
        // Add a normalized feedforward term so requested wheel speed maps directly to motor output.
        double driveFeedforward = optimizedState.speedMetersPerSecond / SwerveConstants.MAX_TRANSLATIONAL_SPEED;
        double driveFeedback = drivePID.calculate(getVelocity(), optimizedState.speedMetersPerSecond);
        double driveOutput = driveFeedforward + driveFeedback;
        double turnOutput = turnPID.calculate(currentAngle.getRadians(), optimizedState.angle.getRadians());

        // Suppress small angle-hunting around setpoint to reduce module twitch.
        double angleErrorRad = MathUtil.angleModulus(optimizedState.angle.getRadians() - currentAngle.getRadians());
        if (Math.abs(angleErrorRad) < Math.toRadians(TURN_DEADBAND_DEG)) {
            turnOutput = 0.0;
        }
        // If the CANcoder is offline/invalid, avoid unstable turn control and still allow drive output.
        if (!canCoderOnline) {
            turnOutput = 0.0;
        }

        driveOutput = MathUtil.clamp(driveOutput, -1.0, 1.0);
        // Kill tiny speed requests to prevent low-speed chatter.
        if (Math.abs(optimizedState.speedMetersPerSecond) < DRIVE_REQUEST_DEADBAND_MPS) {
            driveOutput = 0.0;
        }

        // Do not drive while module heading is still far from target.
        double absAngleErrorDeg = Math.abs(Math.toDegrees(angleErrorRad));
        if (absAngleErrorDeg > DRIVE_SUPPRESS_ANGLE_DEG) {
            driveOutput = 0.0;
        }
        turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);

        lastDriveCommand = driveOutput;
        lastTurnCommand = turnOutput;
        
        turnMotor.set(turnOutput);
        driveMotor.set(driveOutput);
    }

    // var delta = angle.minus(currentAngle);
    // if (Math.abs(delta.getDegrees()) > 90.0) {
    //   speedMetersPerSecond *= -1;
    //   angle = angle.rotateBy(Rotation2d.kPi);
    // }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPos() {
        // Phoenix6 absolute position is in rotations [0,1). Convert to radians and apply module offset.
        double absoluteRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        if (!Double.isFinite(absoluteRotations) || Math.abs(absoluteRotations) > 1.5) {
            return hasValidAngle ? lastValidAngle.getRadians() : 0.0;
        }
        double angleRad = (absoluteRotations * SwerveConstants.TWO_PI) - absoluteOffsetRad;
        lastValidAngle = Rotation2d.fromRadians(MathUtil.angleModulus(angleRad));
        hasValidAngle = true;
        return angleRad;
    }

    public double getAbsoluteEncoderDegrees() {
        return Math.toDegrees(getAbsoluteEncoderPos());
    }

    public int getConfiguredCanCoderId() {
        return absoluteEncoderId;
    }

    public boolean isCanCoderOnline() {
        try {
            double absoluteRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
            return Double.isFinite(absoluteRotations) && Math.abs(absoluteRotations) <= 1.5;
        } catch (Exception e) {
            return false;
        }
    }

    public Rotation2d getAngle() {
        // Wrap to [-pi, pi] to match continuous-input PID configuration.
        return Rotation2d.fromRadians(MathUtil.angleModulus(getAbsoluteEncoderPos()));
    }

    public double getLastDriveCommand() {
        return lastDriveCommand;
    }

    public double getLastTurnCommand() {
        return lastTurnCommand;
    }

    public void stopMotors() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
        lastDriveCommand = 0.0;
        lastTurnCommand = 0.0;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }
}
