// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    //Calls the onStart method in RobotContainer (things which should be done when the robot is turned on)
    m_robotContainer.onStart();
  }

  //Called every 20ms
  @Override
  public void robotPeriodic() {
    //Runs the scheduler
    CommandScheduler.getInstance().run();
  }

  //Called when the robot is disabled
  @Override
  public void disabledInit() {}

  //Called every 20ms when the robot is disabled
  @Override
  public void disabledPeriodic() {}

  //Called when the robot exits disabled mode
  @Override
  public void disabledExit() {}

  //Called when the robot enters autonomous mode
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  //Called every 20ms when the robot is in autonomous mode
  @Override
  public void autonomousPeriodic() {}

  //Called when the robot exits autonomous mode
  @Override
  public void autonomousExit() {}

  //Called when the robot enters teleop mode
  @Override
  public void teleopInit() {
    //This line cancels the autonomous command when teleop starts
    //If you want the autonomous command to continue running in teleop until a button is pressed, 
    //comment this line out
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  //Called every 20ms when the robot is in teleop mode
  @Override
  public void teleopPeriodic() {}

  //Called when the robot exits teleop mode
  @Override
  public void teleopExit() {}


  //Called when the robot enters test mode
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  //Called every 20ms when the robot is in test mode
  @Override
  public void testPeriodic() {}

  //Called when the robot exits test mode
  @Override
  public void testExit() {}

  //Called when the robot enters simulation mode
  @Override
  public void simulationInit () {}

  //Called every 20ms when the robot is in simulation mode
  @Override
  public void simulationPeriodic () {}
}
