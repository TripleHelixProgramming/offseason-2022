// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.drive.Drivetrain;
import frc.robot.drive.commands.JoystickDrive;
import frc.robot.drive.commands.ResetEncoders;
import frc.robot.oi.OI;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer INSTANCE;

  /**
   * @return retrieves the singleton instance of the Robot Container
   */
  public static RobotContainer getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new RobotContainer();
    }
    return INSTANCE;
  }

  // The robot's subsystems
  private final Drivetrain m_robotDrive = new Drivetrain();
  private final PowerDistribution m_PDP = new PowerDistribution(ElectricalConstants.pdpPort, ElectricalConstants.pdpType);
  private final OI m_OI = OI.getInstance();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_OI.setDrivetrain(m_robotDrive);
    m_OI.configureButtonBindings();

    SmartDashboard.putData("Reset Encoders", new ResetEncoders(m_robotDrive));

    // Configure default commands
    setDefaultCommands();

  }

  private void setDefaultCommands() {
    // m_robotDrive.setDefaultCommand(new RelativeOrientation(m_robotDrive));
    m_robotDrive.setDefaultCommand(new JoystickDrive(m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    // return new Command();
  // }
}
