// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spark;


import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

public class SparkMaxSwerveModule {

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SparkMaxSwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
//      int driveEncoderPort,
      int turningEncoderPort,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

//    this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);
    this.m_driveEncoder = m_driveMotor.getEncoder();

//    this.m_turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);
    this.m_turningEncoder = new CANCoder(turningEncoderPort);


    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
//    m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);
    
    // m_driveEncoder returns RPM by default. Use setVelocityConversionFactor() to convert that to meters per second.
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);

    // Set whether drive encoder should be reversed or not
    // Not applicable for brushless motors
//    m_driveEncoder.setInverted(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
//    m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    m_turningEncoder.configSensorDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(Math.toRadians(m_turningEncoder.getPosition())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed (in meters per second?) and angle (in degrees).
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    final var driveOutput =
        m_drivePIDController.calculate(m_driveMotor.get(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getVelocity(), state.angle.getDegrees());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0.0);
    m_turningEncoder.setPosition(0.0);
  }
}
