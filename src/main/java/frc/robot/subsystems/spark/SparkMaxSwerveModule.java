// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spark;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SparkMaxSwerveModule extends SubsystemBase {

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final CANEncoder m_driveEncoder;
    private final CANEncoder m_turningEncoder;

    private final CANCoder m_turningCANCoder;

    // absolute offset for the CANCoder so that the wheels can be aligned when the
    // robot is turned on
    private final Rotation2d m_CANCoderOffset;

    private final CANPIDController m_turningController;
    private final CANPIDController m_driveController;

    // private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kDriveP,
    //         ModuleConstants.kDriveI, ModuleConstants.kDriveD);

    // // Using a TrapezoidProfile PIDController to allow for smooth turning
    // private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(ModuleConstants.kTurningP,
    //         ModuleConstants.kTurningI, ModuleConstants.kTurningD,
    //         new TrapezoidProfile.Constraints(ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
    //                 ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     */
    public SparkMaxSwerveModule(
                        int driveMotorChannel,
                        int turningMotorChannel,
                        int turningCANCoderChannel,
                        double turningCANCoderOffsetDegrees) {

        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        m_turningCANCoder = new CANCoder(turningCANCoderChannel);
        m_turningCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        m_turningCANCoder.setPosition(0);
        m_CANCoderOffset = Rotation2d.fromDegrees(turningCANCoderOffsetDegrees);

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turningMotor.setIdleMode(IdleMode.kCoast);

        // m_driveEncoder returns RPM by default. Use setVelocityConversionFactor() to
        // convert that to meters per second.
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0);
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor);

        m_turningEncoder.setPositionConversionFactor(1.0 / ModuleConstants.kTurnPositionConversionFactor);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        // m_turningPIDController.enableContinuousInput(0, 360);

        m_turningController = m_turningMotor.getPIDController();
        m_driveController = m_driveMotor.getPIDController();

        m_turningController.setP(Constants.ModuleConstants.kTurningP);
        // m_turningController.setI(Constants.ModuleConstants.kTurningI);
        m_turningController.setD(Constants.ModuleConstants.kTurningD);

        // 401 only sets P of the drive PID
        m_driveController.setP(Constants.ModuleConstants.kDriveP);
        // m_driveController.setI(Constants.ModuleConstants.kDriveI);
        // m_driveController.setD(Constants.ModuleConstants.kDriveD);


    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // getPosition() returns the number of cumulative rotations.
        // Convert that to 0.0 to 1.0
        double m1 = m_turningEncoder.getPosition() % 1.0;
        double m2 = (m1 < 0) ? m1 + 1 : m1;

        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m2 * 2 * Math.PI));
    }

    public CANSparkMax getTurnMotor() {
        return m_turningMotor;
    }

    public CANEncoder getTurnEncoder() {
        return m_turningEncoder;
    }

    public CANCoder getTurnCANcoder() {
        return m_turningCANCoder;
    }

    public double getTurnCANcoderAngle() {
        return m_turningCANCoder.getAbsolutePosition();
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed (in meters per second?) and angle (in
     *              degrees).
     */
    public void setDesiredState(SwerveModuleState state) {

        Rotation2d curAngle = Rotation2d.fromDegrees(m_turningCANCoder.getAbsolutePosition());

        SmartDashboard.putNumber("target angle", state.angle.getDegrees());
        SmartDashboard.putNumber("current angle", curAngle.getDegrees());

        double error = curAngle.getDegrees() - state.angle.getDegrees();
        double m01 = ((error + 180) % (360)) - 180;
        double m02 = m01 < -180 ? m01 + 360 : m01;

        final var turnOutput = -0.01 * m02;

        Rotation2d adjustedAngle = new Rotation2d(calculateAdjustedAngle(state.angle.getRadians(), curAngle.getRadians()));
        SmartDashboard.putNumber("adjusted angle", adjustedAngle.getDegrees());

        // m_turningController.setReference(
        //     turnOutput,
        //     ControlType.kPosition
        // );        
        m_turningMotor.set(turnOutput);

        // Calculate the drive motor output from the drive PID controller.
        final var driveOutput = state.speedMetersPerSecond * 0.1; 
        // m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond)
        m_driveMotor.set(driveOutput);

        // double speedRadPerSec = driveOutput / (Constants.ModuleConstants.kWheelDiameterMeters / 2);

        // m_driveController.setReference(driveOutput, ControlType.kVelocity, 0,  
        //     Constants.ModuleConstants.driveFF.calculate(speedRadPerSec));
    }

    //calculate the angle motor setpoint based on the desired angle and the current angle measurement
    // Arguments are in radians.
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;

    }

    public double getDriveDistanceMeters() {
        return m_driveEncoder.getPosition();
    }

    public void resetDistance() {
        m_driveEncoder.setPosition(0.0);
    }

    /** Zeros all the SwerveModule encoders. */
    public void resetEncoders() {
        // Reset the cumulative rotation counts of the SparkMax motors
        m_turningEncoder.setPosition(0.0);

        m_turningCANCoder.setPosition(0.0);
        m_turningCANCoder.configMagnetOffset(m_turningCANCoder.configGetMagnetOffset() - m_turningCANCoder.getAbsolutePosition());
    }
}
