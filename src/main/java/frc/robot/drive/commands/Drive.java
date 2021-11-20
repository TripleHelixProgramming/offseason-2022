// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import frc.robot.drive.Drivetrain;

public abstract class Drive extends CommandBase {

    private double xDot;
    private double yDot;
    private double thetaDot;
    private boolean fieldRelative;
    private ChassisSpeeds chassisSpeeds;

    // The subsystem the command runs on
    public final Drivetrain drivetrain;

    public Drive(Drivetrain subsystem){
        drivetrain = subsystem;
        addRequirements(drivetrain);
    }
 
    @Override
    public void initialize() {
    }
            
    @Override
    public void execute() {
        xDot = getX();
        yDot = getY();
        thetaDot = getTheta();
        fieldRelative = getFieldRelative();

        chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, thetaDot, drivetrain.getHeading())
            : new ChassisSpeeds(xDot, yDot, thetaDot);

        drivetrain.drive(chassisSpeeds);
    }

    abstract public double getX();
    abstract public double getY();
    abstract public double getTheta();
    abstract public boolean getFieldRelative();
}