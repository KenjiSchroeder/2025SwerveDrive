// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.Commands.DriveMotors;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.TestMotorsSubsystem;

import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private DriveSubsystem m_driveSub;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.k_DriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     initSubsystems();    
    // Configure the trigger bindings
    configureBindings();
    configureSmartDashboard();
    configurePathPlanner();
  }

  private void initSubsystems() {
        if(OperatingConstants.k_usingSwerveDrive) {
                m_driveSub = new DriveSubsystem();
                m_driveSub.setDefaultCommand(new RunCommand(
                        () -> m_driveSub.drive(
                                OIConstants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisX), OIConstants.k_DriveDeadband), 
                                OIConstants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisY), OIConstants.k_DriveDeadband), 
                                OIConstants.k_driverRotAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisRot), OIConstants.k_DriveDeadband), 
                                true,
                                "Default / Field Oriented"
                        ), 
                        m_driveSub)
                );
        } else {
                m_driveSub = null;
        }
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void onStart() {
    //If we want to do something when the robot is turned on, we can put it here
    //(Like camera setup or something)
  }
}
