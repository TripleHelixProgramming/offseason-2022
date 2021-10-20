// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final int kPigeonPort = 20;

    public static final class SparkCAN {
      public static final int kRearRightDriveMotorPort = 10;
      public static final int kFrontRightDriveMotorPort = 12;
      public static final int kFrontLeftDriveMotorPort = 22;
      public static final int kRearLeftDriveMotorPort = 24;
  
      public static final int kRearRightTurningMotorPort = 11;  
      public static final int kFrontRightTurningMotorPort = 13;
      public static final int kFrontLeftTurningMotorPort = 23;
      public static final int kRearLeftTurningMotorPort = 25;
    }

    public static final class TalonCAN {
      public static final int kFrontLeftDriveMotorPort = 0;
      public static final int kRearLeftDriveMotorPort = 2;
      public static final int kFrontRightDriveMotorPort = 4;
      public static final int kRearRightDriveMotorPort = 6;
  
      public static final int kFrontLeftTurningMotorPort = 1;
      public static final int kRearLeftTurningMotorPort = 3;
      public static final int kFrontRightTurningMotorPort = 5;
      public static final int kRearRightTurningMotorPort = 7;  
    }

    public static final class Example {
      public static final int kFrontLeftDriveMotorPort = 0;
      public static final int kRearLeftDriveMotorPort = 2;
      public static final int kFrontRightDriveMotorPort = 4;
      public static final int kRearRightDriveMotorPort = 6;

      public static final int kFrontLeftTurningMotorPort = 1;
      public static final int kRearLeftTurningMotorPort = 3;
      public static final int kFrontRightTurningMotorPort = 5;
      public static final int kRearRightTurningMotorPort = 7;

      public static final int[] kFrontLeftDriveEncoderPorts = new int[] {8, 9};
      public static final int[] kRearLeftDriveEncoderPorts = new int[] {10, 11};
      public static final int[] kFrontRightDriveEncoderPorts = new int[] {12, 13};
      public static final int[] kRearRightDriveEncoderPorts = new int[] {14, 15};

      public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
      public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
      public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
      public static final int[] kRearRightTurningEncoderPorts = new int[] {6, 7};  
    }

    public static final class CANCoder {
      // Below provided by SparkMAX motors API.
      // public static final int kFrontLeftDriveEncoderPort = 
      // public static final int kRearLeftDriveEncoderPort =
      // public static final int kFrontRightDriveEncoderPort =
      // public static final int kRearRightDriveEncoderPort =

      public static final int kRearRightTurningEncoderPort = 31;
      public static final int kFrontRightTurningEncoderPort = 33;
      public static final int kFrontLefTurningEncoderPort = 43;
      public static final int kRearLeftTurningEncoderPort = 45;

      public static final double kRearRightTurningEncoderOffset = 180.0;
      public static final double kFrontRightTurningEncoderOffset = 180.0;
      public static final double kFrontLefTurningEncoderOffset = 180.0;
      public static final double kRearLeftTurningEncoderOffset = 180.0;


    }

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    // Units are meters.
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.5715; // 22.5 in
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.6223; // 24.5 in
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    /* I don't know if these are needed. They only appear here in the project. i.e. They are never used.
    public static final double ksVolts = 1.0;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    */
    public static final double kMaxSpeedMetersPerSecond = 3.0;
  }

  public static final class ModuleConstants {

    public static final double kDriveP = 1.0;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.5;

    public static final double kTurningP = 0.005;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 0.000005 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 0.000005 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.09398; // 3.7 in

    // The drive encoder reports in RPM by default. Calculate the conversion factor
    // to make it report in meters per second.
    public static final double kDriveVelocityConversionFactor = (kWheelDiameterMeters * Math.PI) / 60.0;


    public static final double kTurnPositionConversionFactor = 12.8;


    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1.0;

    public static final double kPModuleDriveController = 1.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
