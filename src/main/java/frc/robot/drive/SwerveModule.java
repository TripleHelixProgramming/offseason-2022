// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final DutyCycleEncoder m_throughBoreEncoder;
    
    // absolute offset for the CANCoder so that the wheels can be aligned when the
    // robot is turned on
    private final Rotation2d m_EncoderOffset;

    private final SparkMaxPIDController m_turningController;
    private final SparkMaxPIDController m_driveController; 
    
    private final int turningChannel;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     */
    public SwerveModule(
                        int driveMotorChannel,
                        int turningMotorChannel,
                        int turningCANCoderChannel,
                        double turningCANCoderOffsetDegrees) {
        turningChannel = turningMotorChannel;

        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        m_throughBoreEncoder = new DutyCycleEncoder(turningCANCoderChannel);
        
        m_EncoderOffset = Rotation2d.fromDegrees(turningCANCoderOffsetDegrees);

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turningMotor.setIdleMode(IdleMode.kCoast);

        // m_driveEncoder returns RPM by default. Use setVelocityConversionFactor() to
        // convert that to meters per second.
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0);
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor);

        m_turningEncoder.setPositionConversionFactor(360.0 / ModuleConstants.kTurnPositionConversionFactor);

        m_turningController = m_turningMotor.getPIDController();
        m_driveController = m_driveMotor.getPIDController();

        m_turningController.setFF(0);
        m_turningController.setP(Constants.ModuleConstants.kTurningP);
        m_turningController.setI(0);
        m_turningController.setD(Constants.ModuleConstants.kTurningD);

        m_driveController.setP(Constants.ModuleConstants.kDriveP);
        m_driveController.setD(Constants.ModuleConstants.kDriveD);

        SmartDashboard.putNumber("Current thru bore: " + turningChannel, m_throughBoreEncoder.getAbsolutePosition());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // getPosition() returns the number of cumulative rotations.
        // Convert that to 0.0 to 1.0
        // double m1 = m_turningEncoder.getPosition() % 360.0;
        // double m2 = (m1 < 0) ? m1 + 360 : m1;

        double m2 = (m_turningEncoder.getPosition() % 360 + 360) % 360;

        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m2 * Math.PI / 180));
    }

    public CANSparkMax getTurnMotor() {
        return m_turningMotor;
    }

    public RelativeEncoder getTurnEncoder() {
        return m_turningEncoder;
    }

    public DutyCycleEncoder getTurnCANcoder() {
        return m_throughBoreEncoder;
    }

    public double getThroughBoreEncoderPosition() {
        return m_throughBoreEncoder.getAbsolutePosition() - m_EncoderOffset.getDegrees();
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed (in meters per second?) and angle (in
     *              degrees).
     */
    public void setDesiredState(SwerveModuleState state) {
        Rotation2d curAngle = Rotation2d.fromDegrees(m_turningEncoder.getPosition());

        double delta = deltaAdjustedAngle(state.angle.getDegrees(), curAngle.getDegrees());

        // Calculate the drive motor output from the drive PID controller.
        double driveOutput = state.speedMetersPerSecond;

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        // Rotation2d adjustedAngle = Rotation2d.fromDegrees(delta + curAngle.getDegrees());
        Rotation2d adjustedAngle = Rotation2d.fromDegrees(-500);

        m_turningController.setReference(adjustedAngle.getDegrees(), ControlType.kPosition);        

        SmartDashboard.putNumber("Commanded Velocity", driveOutput);
        SmartDashboard.putNumber("Current position: " + turningChannel, m_turningEncoder.getPosition());
        
        SmartDashboard.putNumber("Commanded position: " + turningChannel, adjustedAngle.getDegrees());

        m_driveController.setReference(driveOutput, ControlType.kVelocity, 0, Constants.ModuleConstants.kDriveFF * driveOutput);
    }

    //calculate the angle motor setpoint based on the desired angle and the current angle measurement
    // Arguments are in radians.
    public double deltaAdjustedAngle(double targetAngle, double currentAngle) {
        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    public double getDriveDistanceMeters() {
        return m_driveEncoder.getPosition();
    }

    public void resetDistance() {
        m_driveEncoder.setPosition(0.0);
    }

    public void syncTurningEncoders() {
        m_turningEncoder.setPosition(getThroughBoreEncoderPosition());
    }
}