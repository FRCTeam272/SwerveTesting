// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drive _drive = Drive.getInstance();
  public static NavXGyro _gyro = NavXGyro.getInstance();

  public XboxController driver;
  public Joystick operator;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    driver = new XboxController(Constants.DriverChannel);
    operator = new Joystick(Constants.OperatorChannel);

    CommandScheduler.getInstance().setDefaultCommand(_drive,
    new DriveCommand(_drive, driver, _gyro));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // EXAMPLE of using a subsystem
    /*
      new JoystickButton(driver, 1).whenPressed(() -> _intake.autoLower());

      new JoystickButton makes a new Button for you to inspect
        passing it the controller and the targeting button will allow you to add on press
      
      the rest is called an anonymous inner function, this is basically an undefined item that just says when button pressed do this function
    */

    

    
  }
}
