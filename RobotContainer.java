package frc.robot;

import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.TurnToAngle;
//import frc.robot.commands.DriveToDis;
//import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  GenericHID controller = new GenericHID(Constants.OIConstants.kDriverControllerPort);
  Drive drive = new Drive();

  public RobotContainer() {
    drive.setDefaultCommand(new ArcadeDriveCmd(drive, () -> controller.getRawAxis(1), () -> controller.getRawAxis(2)));
    configureBindings();
  }

 
  private void configureBindings() {
    new JoystickButton(controller, 1).onTrue(new TurnToAngle(45, drive));
    new JoystickButton(controller, 2).onTrue(new TurnToAngle(90, drive));
    new JoystickButton(controller, 3).onTrue(new TurnToAngle(180, drive));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}