// Copyright (c) Triple Helix Robotics

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public final class Constants {
  public static final class ElectricalConstants {
      public static final ModuleType pdpType = ModuleType.kCTRE;
      public static final int pdpPort = 0;

      //port #s for the encoders
      public static final int kRearRightTurningEncoderPort = 0;
      public static final int kFrontRightTurningEncoderPort = 1;
      public static final int kFrontLeftTurningEncoderPort = 2;
      public static final int kRearLeftTurningEncoderPort = 3;

      //port #s for the drive motors
      public static final int kRearRightDriveMotorPort = 10;
      public static final int kFrontRightDriveMotorPort = 12;
      public static final int kFrontLeftDriveMotorPort = 22;
      public static final int kRearLeftDriveMotorPort = 24;
  
      //port #s for the turning motors
      public static final int kRearRightTurningMotorPort = 11;  
      public static final int kFrontRightTurningMotorPort = 13;
      public static final int kFrontLeftTurningMotorPort = 23;
      public static final int kRearLeftTurningMotorPort = 25;

      public static final int kGyroPort = 20;
  }

  public static final class DriveConstants {

    // Define the conventional order of our modules when putting them into arrays
    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 1;
    public static final int REAR_LEFT = 2;
    public static final int REAR_RIGHT = 3;
    
    public static final double kRearRightTurningEncoderOffset = 358.88184;
    public static final double kFrontRightTurningEncoderOffset = 0.0;
    public static final double kFrontLeftTurningEncoderOffset = 0.0;
    public static final double kRearLeftTurningEncoderOffset = 0.0;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    // Units are meters.
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.5715; // 22.5 in
    
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.6223; // 24.5 in

    // Units are meters per second
    public static final double kMaxTranslationalVelocity = 0.75; //max 4.5

    // Units are radians per second
    public static final double kMaxRotationalVelocity = 5.0; //max 5.0

    //The locations for the modules must be relative to the center of the robot. 
    // Positive x values represent moving toward the front of the robot 
    // Positive y values represent moving toward the left of the robot.
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),   // front left
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),  // front right
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),  // rear left
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)  // rear right
            );

    public static final boolean kGyroReversed = false;

    // Is the joystick drive in field relative mode.
    public static final boolean kFieldRelative = true;
  }

  public static final class ModuleConstants {

    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveFF = 2.96;

    public static final double kSteerP = -0.01;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.0;
    public static final double kSteerFF = 0.0;

    public static final double kMaxSpeedMetersPerSecond = 3.0; // meters

    public static final double kWheelDiameterMeters = 0.0762; // 3 in

    // Gear reduction (unitless) between the drive motor and the wheel
    public static final double kDriveGearRatio = 5.5;

    // The drive encoder reports in RPM by default. Calculate the conversion factor
    // to make it report in meters per second.
    public static final double kDriveConversionFactor = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    // Gear reduction (unitless) between the steering motor and the module azimuth
    // Stage 1 - REV Ultraplanetary nominal "4:1" stage, actual ratio 29:84
    // Stage 2 - REV Ultraplanetary nominal "3:1" stage, actual ratio 21:76
    // Stage 3 - 14:62
    public static final double kTurnPositionConversionFactor = 46.42;

    public static final int kNominalVoltage = 12;
    public static final int kDriveCurrentLimit = 60;
    public static final int kSteerCurrentLimit = 25;
  }
}