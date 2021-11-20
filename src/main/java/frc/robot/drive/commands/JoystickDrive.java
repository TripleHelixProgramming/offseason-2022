// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.drive.Drivetrain;

import frc.lib.Curve;
import frc.lib.LinCurve;

public class JoystickDrive extends Drive {

    double xyJoyScale = DriveConstants.kMaxTranslationalVelocity;
    Curve xyJoyMap = new LinCurve(0.0, xyJoyScale, 0.4);

    double thetaJoyScale = DriveConstants.kMaxRotationalVelocity;
    Curve thetaJoyMap = new LinCurve(0.0, thetaJoyScale, 0.4);

    public JoystickDrive(Drivetrain subsystem){
        super(subsystem);
    }
 
    @Override
    public double getX() {
        double xRaw = RobotContainer.getInstance().getTranslateXJoystick();
        return xyJoyMap.calculateMappedVal(xRaw);
    }

    @Override
    public double getY() {
        double yRaw = RobotContainer.getInstance().getTranslateYJoystick();
        return xyJoyMap.calculateMappedVal(yRaw);
    }

    @Override
    public double getTheta() {
        double thetaRaw = RobotContainer.getInstance().getRotateJoystick();
        return thetaJoyMap.calculateMappedVal(thetaRaw);
    }

    @Override
    public boolean getFieldRelative() {
        return true;
    }
}