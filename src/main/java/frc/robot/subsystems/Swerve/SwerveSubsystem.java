package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveSubsystem extends SubsystemBase {
    
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private final PIDController[] drivePIDControllers;
    private final ProfiledPIDController[] turnPIDControllers;
    // private final PIDController[] turnPIDControllers;

    // Properties for Field oriented driving
    private Gyro fieldGyro;
    
    // for resetting gyro
    private Rotation2d gyroOffset = new Rotation2d();

    private SwerveDriveOdometry odometry;
    private Field2d field = new Field2d();
    private ChassisSpeeds lastRequestedRobotRelativeSpeeds = new ChassisSpeeds();

    public SwerveSubsystem() {
        fieldGyro = new Gyro();
        gyroOffset = new Rotation2d();
        // Initialize the swerve modules
        modules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.IDS[0], SwerveConstants.IDS[1], SwerveConstants.IDS[2], SwerveConstants.DRIVE_MOTOR_INVERTED[0], SwerveConstants.TURN_MOTOR_INVERTED[0], SwerveConstants.CANCODER_OFFSETS_RAD[0]),
            // TEMP: only CANcoder 1 is online; use ID 1 for all modules until 2/3/4 are fixed.
            new SwerveModule(SwerveConstants.IDS[3], SwerveConstants.IDS[4], SwerveConstants.IDS[5], SwerveConstants.DRIVE_MOTOR_INVERTED[1], SwerveConstants.TURN_MOTOR_INVERTED[1], SwerveConstants.CANCODER_OFFSETS_RAD[1]),
            new SwerveModule(SwerveConstants.IDS[6], SwerveConstants.IDS[7], SwerveConstants.IDS[8], SwerveConstants.DRIVE_MOTOR_INVERTED[2], SwerveConstants.TURN_MOTOR_INVERTED[2], SwerveConstants.CANCODER_OFFSETS_RAD[2]),
            new SwerveModule(SwerveConstants.IDS[9], SwerveConstants.IDS[10], SwerveConstants.IDS[11], SwerveConstants.DRIVE_MOTOR_INVERTED[3], SwerveConstants.TURN_MOTOR_INVERTED[3], SwerveConstants.CANCODER_OFFSETS_RAD[3]),
            // Original IDs for these 3 lines were IDS[5], IDS[8], IDS[11].
        };
        // Initalize kinematic objects
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[0], SwerveConstants.MODULE_TRANSLATIONS[1]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[2], SwerveConstants.MODULE_TRANSLATIONS[3]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[4], SwerveConstants.MODULE_TRANSLATIONS[5]),
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[6], SwerveConstants.MODULE_TRANSLATIONS[7])
        );
        // initialize PIDidontrollers 
        drivePIDControllers = new PIDController[] {
            new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2])
        };
        turnPIDControllers = new ProfiledPIDController[] {
             new ProfiledPIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2], new TrapezoidProfile.Constraints(SwerveConstants.ANGLE_MAX_VELOCITY, SwerveConstants.ANGLE_MAX_ACCELERATION)),
             new ProfiledPIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2], new TrapezoidProfile.Constraints(SwerveConstants.ANGLE_MAX_VELOCITY, SwerveConstants.ANGLE_MAX_ACCELERATION)),
             new ProfiledPIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2], new TrapezoidProfile.Constraints(SwerveConstants.ANGLE_MAX_VELOCITY, SwerveConstants.ANGLE_MAX_ACCELERATION)),
             new ProfiledPIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2], new TrapezoidProfile.Constraints(SwerveConstants.ANGLE_MAX_VELOCITY, SwerveConstants.ANGLE_MAX_ACCELERATION))  
        };
        for (ProfiledPIDController p : turnPIDControllers) {
            p.enableContinuousInput(-Math.PI, Math.PI);
        };

        // For Field driving 
        odometry = new SwerveDriveOdometry(kinematics, fieldGyro.getRotation2d(), getPositions());

        RobotConfig config;
        try{

        config = RobotConfig.fromGUISettings();

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                // PathPlanner provides robot-relative speeds in SI units.
                // Send them directly to avoid applying speed scaling a second time.
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]), // Translation PID constants
                        new PIDConstants(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2]) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }
    }


        public void resetGyro(double degrees) { 
            fieldGyro.resetGyro(degrees);
        }

    @Override 
    public void periodic () {
        /*
        double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-sublime");
        SmartDashboard.putNumber("limelight/x pos", postions[2]);
        SmartDashboard.putNumber("limelight/y pos", postions[0]);
        SmartDashboard.putNumber("limelight/rot pos", postions[4]);
        */

        // pose stuff 
        odometry.update(fieldGyro.getRotation2d(), getPositions());
        field.setRobotPose(getPose());
        logStates(); 
    }


    public void drive(double y, double x, double theta, boolean fieldRelative, boolean alignLimelight, boolean resetGyro) {
        ChassisSpeeds newDesiredSpeeds; 
        
        if (alignLimelight) { 
            newDesiredSpeeds = new ChassisSpeeds(y, x, theta);
        } else { 
            newDesiredSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * y, 
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * x,
            SwerveConstants.MAX_ROTATIONAL_SPEED * theta
        );
        }

        // reset gyro button
        if (resetGyro) {
            gyroOffset = fieldGyro.getRotation2d();
        }

        // implementing field logic
        if (fieldRelative) {
            driveFieldRelative(newDesiredSpeeds);
        }
        else {
            driveRobotRelative(newDesiredSpeeds);
        }

        // driveRobotRelative(newDesiredSpeeds);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    } 

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(fieldGyro.getRotation2d(), getPositions(), pose);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        Rotation2d drivingAngle = fieldGyro.getRotation2d().minus(gyroOffset);
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, drivingAngle));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        lastRequestedRobotRelativeSpeeds = robotRelativeSpeeds;

        final double linearDeadbandMps = 0.02;
        final double angularDeadbandRadPerSec = 0.02;

        // When no chassis motion is requested, hold module angles to prevent idle hunting.
        if (Math.abs(robotRelativeSpeeds.vxMetersPerSecond) < linearDeadbandMps
                && Math.abs(robotRelativeSpeeds.vyMetersPerSecond) < linearDeadbandMps
                && Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond) < angularDeadbandRadPerSec) {
            stop();
            return;
        }

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        // add constant for max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.MAX_TRANSLATIONAL_SPEED);

        modules[0].setTargetState(targetStates[0], drivePIDControllers[0], turnPIDControllers[0]);
        modules[1].setTargetState(targetStates[1], drivePIDControllers[1], turnPIDControllers[1]);
        modules[2].setTargetState(targetStates[2], drivePIDControllers[2], turnPIDControllers[2]);
        modules[3].setTargetState(targetStates[3], drivePIDControllers[3], turnPIDControllers[3]);
    //    for (int i = 0; i<4; i++) {
    //     modules[i].setTargetState(targetStates[i], drivePIDControllers[i], turnPIDControllers[i]);
    //    }
    }


    public SwerveModuleState[] getStates() {
        SwerveModuleState[] currentStates = {
            modules[0].getState(), 
            modules[1].getState(), 
            modules[2].getState(), 
            modules[3].getState(), 
        };
        return currentStates;
    }

    public void logStates() {
        double maxAbsDriveCommand = Math.max(
                Math.max(Math.abs(modules[0].getLastDriveCommand()), Math.abs(modules[1].getLastDriveCommand())),
                Math.max(Math.abs(modules[2].getLastDriveCommand()), Math.abs(modules[3].getLastDriveCommand())));

        boolean requestedMotion =
                Math.abs(lastRequestedRobotRelativeSpeeds.vxMetersPerSecond) > 0.10
                || Math.abs(lastRequestedRobotRelativeSpeeds.vyMetersPerSecond) > 0.10
                || Math.abs(lastRequestedRobotRelativeSpeeds.omegaRadiansPerSecond) > 0.10;
        boolean commandReachingModules = maxAbsDriveCommand > 0.12;

        SmartDashboard.putBoolean("Swerve/Diag/RequestedMotion", requestedMotion);
        SmartDashboard.putBoolean("Swerve/Diag/CommandReachingModules", commandReachingModules);
        SmartDashboard.putNumber("Swerve/Diag/MaxDriveCmdAbs", maxAbsDriveCommand);
    }
    
    /**
     * Stop driving (zero wheel speeds) but keep current wheel angles.
     */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stopMotors();
        }
    }

    /**
     * Stop and set modules to an X pattern (wheels at +/-45deg) to resist being pushed.
     */
    public void stopWithX() {
        Rotation2d plus45 = Rotation2d.fromDegrees(45.0);
        Rotation2d minus45 = Rotation2d.fromDegrees(-45.0);
        SwerveModuleState[] xStates = new SwerveModuleState[4];
        // Assuming module order: 0=FL,1=FR,2=BL,3=BR
        xStates[0] = new SwerveModuleState(0.0, plus45);
        xStates[1] = new SwerveModuleState(0.0, minus45);
        xStates[2] = new SwerveModuleState(0.0, minus45);
        xStates[3] = new SwerveModuleState(0.0, plus45);
        setStates(xStates);
    }
    
}
