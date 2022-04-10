// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import java.io.FileInputStream;
import java.io.FileNotFoundException;

import org.json.*;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  JSONObject config;

  /** runs at zero packet */
  @Override
  public void robotInit() {
    try{
      generateConfiguration();
    } catch (FileNotFoundException ex){
      System.out.println(ex.getMessage());
    }
    
  }

  private void generateConfiguration() throws FileNotFoundException{
    config = new JSONObject(
      new JSONTokener(
        new FileInputStream("config.json")
      )
    );
  }

  /** runs every n+1 packet */
  @Override
  public void robotPeriodic() {
    
  }

  /** First Disabled */
  @Override
  public void disabledInit() {}

  /** every disabled */
  @Override
  public void disabledPeriodic() {}

  /** runs 1st auto packet/
  @Override
  public void autonomousInit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** runs first init */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    
  }

  @Override
  public void testPeriodic() {}
}
