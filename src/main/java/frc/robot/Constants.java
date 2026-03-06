// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OIConstants {

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DEADBAND = 0.15;
    public static final double TRANSLATION_EXPO = 2.0;
    public static final double ROTATION_EXPO = 2.0;
    public static final double TELEOP_TRANSLATION_SCALE = 1.0;
    public static final double TELEOP_ROTATION_SCALE = 1.0;
    public static final double TRANSLATION_SLEW_RATE = 6.0;
    public static final double ROTATION_SLEW_RATE = 8.0;
  }

  public static class SwerveConstants {

    // Motor Controller and Encoder Configuration
    public static final double VOLTAGE_COMPENSATION = 10;
    public static final int CURRENT_LIMIT = 40;
    public static final double RAMP_RATE = 0.20;
    public static final double WHEEL_DIAMETER_IN = 4;
    public static final double WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN*Math.PI;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double INCHES_PER_METER = 39.3701;
    public static final double TWO_PI = 2.0 * Math.PI;
    // convert native units of rpm to meters per second
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO / INCHES_PER_METER / 60;
    // convert native motor rotations to wheel meters
    public static final double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO / INCHES_PER_METER;

  // IDs
  // Format for each module: {driveMotorId, turningMotorId, canCoderId}
  // Updated to match hardware: drive motor = 11, turn motor = 12, cancoder = 1 for module 0
  public static final int[] IDS = {11, 12, 1, 21, 22, 2, 31, 32, 3, 41, 42, 4};
  // Keep module/electrical order aligned with IDS array above: FL, FR, BL, BR.
  public static final boolean[] DRIVE_MOTOR_INVERTED = {false, false, false, false};
  public static final boolean[] TURN_MOTOR_INVERTED = {false, false, false, false};
  // CANcoder mechanical zero offsets in radians (module order: FL, FR, BL, BR).
  // Values derived from current on-robot readings so current module pose is treated as zero.
  public static final double[] CANCODER_OFFSETS_RAD = {1.47262, 1.56773, -1.56926, -2.66299};

    // PID Values
    
    
    public static final double[] DRIVE_PID_VALUES = {0.015, 0.0, 0.0};
    public static final double[] TURN_PID_VALUES = {0.35, 0.0, 0.005};

    public static final double ANGLE_MAX_VELOCITY = 4.0;
    public static final double ANGLE_MAX_ACCELERATION = 12.0;
    

    // public static final double PID_RANGE = 0.9;
    // 18.7452 m 
    public static final double CHASSIS_LENGTH = Units.inchesToMeters(27.0);
    
    public static final double CHASSIS_WIDTH = Units.inchesToMeters(27.0);
    public static final double[] MODULE_TRANSLATIONS = {
      CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2, 
      CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
    };

    public static final double DRIVE_BASE_RADIUS = Math.hypot(CHASSIS_LENGTH / 2.0, CHASSIS_WIDTH / 2.0);

    
    // Conservative L1 baseline; tune upward after validation.
    public static final double MAX_TRANSLATIONAL_SPEED = 4.5;
    public static final double MAX_ROTATIONAL_SPEED = MAX_TRANSLATIONAL_SPEED / DRIVE_BASE_RADIUS;


    // Swerve Module Location Constants
    // each module is Math.sqrt(2) * Units.inchesToMeters(23) away from the center

  }

}
