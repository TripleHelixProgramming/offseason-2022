package frc.robot.oi;

import static com.team2363.utilities.ControllerMap.*;

import com.team2363.utilities.ControllerMap;
import com.team2363.utilities.ControllerPatroller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.Drivetrain;
import frc.robot.drive.commands.ResetEncoders;
import frc.robot.drive.commands.ZeroHeading;

public class OI {
    private static final String DRIVER = "Xbox";
    private static final int DRIVER_PORT = 0;
    private static final String OPERATOR = "P";
    private static final int OPERATOR_PORT = 1;

    private static final String RADIO_MASTER = "TX16S";

    ControllerPatroller cp;
    Drivetrain dt;

    private Joystick driver;
    private Joystick operator;

    public OI() {
        cp = ControllerPatroller.getPatroller();

        driver = cp.get(DRIVER, DRIVER_PORT);
        
        operator = cp.get(OPERATOR, OPERATOR_PORT);
    }

    public void setDrivetrain(Drivetrain dt) {
      this.dt = dt;
    }

    public void configureButtonBindings() {

      new JoystickButton(driver, ControllerMap.X_BOX_LOGO_LEFT).whenPressed(new ZeroHeading(dt));

      // Below moved to a button on ShuffleBoard.
//      new JoystickButton(driver, ControllerMap.X_BOX_LOGO_RIGHT).whenPressed(new ResetEncoders(dt));

      new JoystickButton(driver, ControllerMap.X_BOX_A);
      new JoystickButton(driver, ControllerMap.X_BOX_B);
      new JoystickButton(driver, ControllerMap.X_BOX_X);
      new JoystickButton(driver, ControllerMap.X_BOX_Y);

      // Set bindings specific to the RadioMaster TX16S controller.
      if (cp.find(RADIO_MASTER).isPresent()) {
        SmartDashboard.putBoolean("RadioMaster Found", true);
        new JoystickButton(driver, 12).whenPressed(new ZeroHeading(dt));
      }

    }

  }
