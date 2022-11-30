// Copyright (c) Triple Helix Robotics

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final DutyCycleEncoder throughBoreEncoder;
    
    private final Rotation2d encoderOffset;

    private final SparkMaxPIDController steerController;
    private final SparkMaxPIDController driveController; 
    
    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   CAN ID for the drive motor controller
     * @param steerMotorChannel   CAN ID for the steer motor controller
     * @param steerEncoderChannel   DIO port for the absolute encoder on the steer axis
     * @param steerEncoderOffset   Rotational transform between absolute encoder and wheel
     */
    public SwerveModule(
                        int driveMotorChannel,
                        int steerMotorChannel,
                        int steerEncoderChannel,
                        Rotation2d steerEncoderOffset) {

        encoderOffset = steerEncoderOffset;

        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorChannel, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setIdleMode(IdleMode.kCoast);

        driveMotor.enableVoltageCompensation(ModuleConstants.kNominalVoltage);
        steerMotor.enableVoltageCompensation(ModuleConstants.kNominalVoltage);

        driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);
        steerMotor.setSmartCurrentLimit(ModuleConstants.kSteerCurrentLimit);

        // driveEncoder returns RPM by default. Use setVelocityConversionFactor() to
        // convert that to meters per second.
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0);
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor);

        steerEncoder.setPositionConversionFactor(360.0 / ModuleConstants.kTurnPositionConversionFactor);

        driveController = driveMotor.getPIDController();
        steerController = steerMotor.getPIDController();

        driveController.setP(ModuleConstants.kDriveP);
        driveController.setI(ModuleConstants.kDriveI);
        driveController.setD(ModuleConstants.kDriveD);
        driveController.setFF(ModuleConstants.kDriveFF);

        steerController.setP(ModuleConstants.kSteerP);
        steerController.setI(ModuleConstants.kSteerI);
        steerController.setD(ModuleConstants.kSteerD);
        steerController.setFF(ModuleConstants.kSteerFF);

        driveMotor.burnFlash();
        steerMotor.burnFlash();

        throughBoreEncoder = new DutyCycleEncoder(steerEncoderChannel);
        steerEncoder.setPosition(getThroughBorePosition());
    }

    public double getThroughBorePosition() {
        // Through bore encoder is in rotations, convert it to degrees.
        return throughBoreEncoder.getAbsolutePosition() * 360 - encoderOffset.getDegrees();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getThroughBorePosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed (in meters per second?) and angle (in
     *              degrees).
     */
    public void setDesiredState(SwerveModuleState state) {
        double currentAngle = steerEncoder.getPosition();
        double delta = deltaAdjustedAngle(state.angle.getDegrees(), currentAngle);
        double driveOutput = state.speedMetersPerSecond;

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        double adjustedAngle = delta + currentAngle;

        SmartDashboard.putNumber("Commanded Velocity" + steerMotor.getDeviceId(), driveOutput);
        SmartDashboard.putNumber("Current position: " + steerMotor.getDeviceId(), steerEncoder.getPosition());
        SmartDashboard.putNumber("Commanded position: " + steerMotor.getDeviceId(), adjustedAngle);

        steerController.setReference(adjustedAngle, ControlType.kPosition);        
        driveController.setReference(driveOutput, ControlType.kVelocity);
    }

    // Compute motor angular setpoint from desired and current angles.
    public double deltaAdjustedAngle(double targetAngle, double currentAngle) {
        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    // Units: meters
    public double getDriveDistance() {
        return driveEncoder.getPosition();
    }

    public void resetDistance() {
        driveEncoder.setPosition(0.0);
    }
}
