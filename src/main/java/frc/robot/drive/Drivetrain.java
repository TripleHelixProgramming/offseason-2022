// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Preferences;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectricalConstants;

@SuppressWarnings("PMD.ExcessiveImports")
public class Drivetrain extends SubsystemBase {

  private Preferences prefs = Preferences.getInstance();
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          ElectricalConstants.kFrontLeftDriveMotorPort,
          ElectricalConstants.kFrontLeftTurningMotorPort,
          ElectricalConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLeftTurningEncoderOffset
          );

  private final SwerveModule m_frontRight =
      new SwerveModule(
          ElectricalConstants.kFrontRightDriveMotorPort,
          ElectricalConstants.kFrontRightTurningMotorPort,
          ElectricalConstants.kFrontRightTurningEncoderPort,
          DriveConstants.kFrontRightTurningEncoderOffset
          );

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          ElectricalConstants.kRearLeftDriveMotorPort,
          ElectricalConstants.kRearLeftTurningMotorPort,
          ElectricalConstants.kRearLeftTurningEncoderPort,
          DriveConstants.kRearLeftTurningEncoderOffset
          );

  private final SwerveModule m_rearRight =
      new SwerveModule(
          ElectricalConstants.kRearRightDriveMotorPort,
          ElectricalConstants.kRearRightTurningMotorPort,
          ElectricalConstants.kRearRightTurningEncoderPort,
          DriveConstants.kRearRightTurningEncoderOffset
          );

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

  // The gyro sensor
  private final AHRS m_ahrs = new AHRS();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  //target pose and controller
  Pose2d m_targetPose;
    
  /** Creates a new DriveSubsystem. */
  public Drivetrain() {

    // Zero out the gyro.
    m_ahrs.zeroYaw();

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getHeading());

    for (SwerveModule module: modules) {
      module.resetDistance();
      module.syncTurningEncoders();
    }
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
    
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    
    // SmartDashboard.putNumber("FrontLeft State Velocity", modules[0].getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("FrontLeft State Angle", modules[0].getState().angle.getDegrees());

    // SmartDashboard.putNumber("FrontRight Velocity", modules[1].getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("FrontRight Angle", modules[1].getState().angle.getDegrees());

    // SmartDashboard.putNumber("RearLeft Velocity", modules[2].getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("RearLeft Angle", modules[2].getState().angle.getDegrees());

    // SmartDashboard.putNumber("RearRight Velocity", modules[3].getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("RearRight Angle", modules[3].getState().angle.getDegrees());
    
    SmartDashboard.putNumber("currentX", getPose().getX());
    SmartDashboard.putNumber("currentY", getPose().getY());
    SmartDashboard.putNumber("currentAngle", getPose().getRotation().getRadians());
    // SmartDashboard.putNumber("targetPoseAngle", m_targetPose.getRotation().getRadians());

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
   * Method to rotate the relative orientation of the target pose at a given rate.
   *
   * @param deltaTheta How much to rotate the target orientation per loop.
   */
  public void rotateRelative(Rotation2d deltaTheta) {
    Transform2d transform = new Transform2d(new Translation2d(), deltaTheta);
    m_targetPose = m_targetPose.transformBy(transform);
  }

  /**
   * Method to drive the robot with given velocities.
   *
   * @param speeds ChassisSpeeds object with the desired chassis speeds [m/s and rad/s].
   */
  @SuppressWarnings("ParameterName")
  public void drive(ChassisSpeeds speeds, ChassisSpeeds percents) {

    if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
      brake();
      return;
    }

    //double xDot, double yDot, double thetaDot, boolean fieldRelative
    
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
           
    normalizeDrive(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond, percents);
    setModuleStates(swerveModuleStates);
  }

  public void brake() {
    for (SwerveModule module : modules) {
      module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
    }
  }

  public void normalizeDrive(SwerveModuleState[] desiredStates, 
                                      double maxVelocity,
                                      ChassisSpeeds percents) {

    double x = percents.vxMetersPerSecond;
    double y = percents.vyMetersPerSecond;
    double theta = percents.omegaRadiansPerSecond;

    // Find the how fast the fastest spinning drive motor is spinning                                       
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : desiredStates) {
      if (Math.abs(moduleState.speedMetersPerSecond) > realMaxSpeed) {
        realMaxSpeed = Math.abs(moduleState.speedMetersPerSecond);
      }
    }
    
    double k = Math.max(Math.hypot(x, y), Math.abs(theta));
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
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, prefs.getDouble("kMaxSpeedMetersPerSecond",DriveConstants.kMaxSpeedMetersPerSecond));

        for (int i = 0; i <= 3; i++) {
          modules[i].setDesiredState(desiredStates[i]);
        }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getState();
    }

    return states;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    for (SwerveModule module: modules) {
      module.resetDistance();
    }
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_ahrs.zeroYaw();
    m_targetPose = new Pose2d(new Translation2d(), new Rotation2d());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return m_ahrs.getRotation2d();
  }
}