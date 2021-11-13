// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spark;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.util.Arrays;
import java.util.Collections;

// import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("PMD.ExcessiveImports")
public class SparkDriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SparkMaxSwerveModule m_frontLeft =
      new SparkMaxSwerveModule(
          DriveConstants.SparkCAN.kFrontLeftDriveMotorPort,
          DriveConstants.SparkCAN.kFrontLeftTurningMotorPort,
          DriveConstants.CANCoder.kFrontLefTurningEncoderPort,
          DriveConstants.CANCoder.kFrontLefTurningEncoderOffset
          );

  private final SparkMaxSwerveModule m_frontRight =
      new SparkMaxSwerveModule(
          DriveConstants.SparkCAN.kFrontRightDriveMotorPort,
          DriveConstants.SparkCAN.kFrontRightTurningMotorPort,
          DriveConstants.CANCoder.kFrontRightTurningEncoderPort,
          DriveConstants.CANCoder.kFrontRightTurningEncoderOffset
          );

  private final SparkMaxSwerveModule m_rearLeft =
      new SparkMaxSwerveModule(
          DriveConstants.SparkCAN.kRearLeftDriveMotorPort,
          DriveConstants.SparkCAN.kRearLeftTurningMotorPort,
          DriveConstants.CANCoder.kRearLeftTurningEncoderPort,
          DriveConstants.CANCoder.kRearLeftTurningEncoderOffset
          );

  private final SparkMaxSwerveModule m_rearRight =
      new SparkMaxSwerveModule(
          DriveConstants.SparkCAN.kRearRightDriveMotorPort,
          DriveConstants.SparkCAN.kRearRightTurningMotorPort,
          DriveConstants.CANCoder.kRearRightTurningEncoderPort,
          DriveConstants.CANCoder.kRearRightTurningEncoderOffset
          );

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();
  // private final PigeonIMU m_pigeon = new PigeonIMU(DriveConstants.kPigeonPort);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public SparkDriveSubsystem() {

    // Zero out the gyro.
    m_gyro.calibrate();
    m_gyro.reset();

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getHeading());

    m_frontLeft.resetDistance();
    m_frontRight.resetDistance();
    m_rearLeft.resetDistance();
    m_rearRight.resetDistance();

    m_frontLeft.syncTurningEncoders();
    m_frontRight.syncTurningEncoders();
    m_rearLeft.syncTurningEncoders();
    m_rearRight.syncTurningEncoders();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getHeading(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());

    SparkMaxSwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
    
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());

    // SmartDashboard.putNumber("FrontLeft State Raw Reading", modules[0].getTurnEncoder().getPosition());
    // SmartDashboard.putNumber("FrontLeft Adjusted Angle", modules[0].adjustedAngle.getDegrees());
    
    SmartDashboard.putNumber("FrontLeft State Velocity", modules[0].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("FrontLeft State Angle", modules[0].getState().angle.getDegrees());

    SmartDashboard.putNumber("FrontRight Velocity", modules[1].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("FrontRight Angle", modules[1].getState().angle.getDegrees());

    SmartDashboard.putNumber("RearLeft Velocity", modules[2].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("RearLeft Angle", modules[2].getState().angle.getDegrees());

    SmartDashboard.putNumber("RearRight Velocity", modules[3].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("RearRight Angle", modules[3].getState().angle.getDegrees());

    // SmartDashboard.putNumber("FronLeft Turning CANcoder Mag Offset", modules[0].getTurnCANcoder().configGetMagnetOffset());
    // SmartDashboard.putNumber("FronLeft Turning CANcoder Abs Position", modules[0].getTurnCANcoder().getAbsolutePosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getHeading());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xScale, double yScale, double rotScale, boolean fieldRelative) {

    double xDot = xScale * DriveConstants.kMaxTranslationalVelocity;
    double yDot = yScale * DriveConstants.kMaxTranslationalVelocity;
    double omega = rotScale * DriveConstants.kMaxRotationalVelocity;

    ChassisSpeeds speeds = fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, omega, getHeading())
                            : new ChassisSpeeds(xDot, yDot, omega);
    
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
           
    normalizeDrive(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond, xScale, yScale, rotScale);
    setModuleStates(swerveModuleStates);
  }

  public void normalizeDrive(SwerveModuleState[] desiredStates, 
                                      double maxVelocity,
                                      double x,
                                      double y,
                                      double theta) {
    double realMaxSpeed = 0.0;
    for (SwerveModuleState module : desiredStates) {
      if (Math.abs(module.speedMetersPerSecond) > realMaxSpeed) {
        realMaxSpeed = Math.abs(module.speedMetersPerSecond);
      }
    }
    double k = Math.max(Math.sqrt(x*x + y*y), Math.abs(theta));
    if (realMaxSpeed != 0.0) {
      for (SwerveModuleState moduleState : desiredStates) {
        moduleState.speedMetersPerSecond *= (k * maxVelocity / realMaxSpeed);
      }
    }
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.normalizeWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };

    return states;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
//    m_pigeon.setYaw(0.0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
    // double[] ypr_deg = {0.0, 0.0, 0.0};
    // m_pigeon.getYawPitchRoll(ypr_deg);
    // return new Rotation2d(Math.toRadians(ypr_deg[0]));
  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
