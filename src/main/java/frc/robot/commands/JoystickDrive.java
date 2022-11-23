// Copyright (c) Triple Helix Robotics

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class JoystickDrive extends CommandBase {
    private final SwerveDrive drivetrain;
    private DoubleSupplier x, y, theta;

    public JoystickDrive(SwerveDrive drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta){
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.theta = theta;
        addRequirements(drivetrain);
    }
 
    @Override
    public void initialize() {
    }
            
    @Override
    public void execute() {
        double vx = x.getAsDouble() * DriveConstants.kMaxTranslationalVelocity;
        double vy = y.getAsDouble() * DriveConstants.kMaxTranslationalVelocity;
        double omega = theta.getAsDouble() * DriveConstants.kMaxRotationalVelocity;

        ChassisSpeeds velocity = DriveConstants.kFieldRelative
                                 ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drivetrain.getHeading())
                                 : new ChassisSpeeds(vx, vy, omega);

        ChassisSpeeds percent = new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble());

        drivetrain.drive(velocity, percent);
    }
}