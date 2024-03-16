// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DriverControls;
import frc.robot.commands.OperatorControls;
import frc.robot.commands.AutoRoutine;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.motor.Motors;
import frc.robot.subsystems.LimelightSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.subsystems.AutoDriveSubsystem;


public class RobotContainer {
  DriveSubsystem drive = new DriveSubsystem();
  RobotSubsystem robot = new RobotSubsystem();
  private final AutoDriveSubsystem auto = new AutoDriveSubsystem();
  // LimelightSubsystem limelight = new LimelightSubsystem();
  
  XboxController driverController = new XboxController(Constants.DRIVE_CONTROL_PORT);
  XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROL_PORT);
  GenericHID buttonBoard = new GenericHID(Constants.BUTTON_BOARD_PORT);
  
  DriverControls driver;
  OperatorControls operator;
  Motors encodeTest;

  //private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drive.setDefaultCommand(new DriverControls(drive, robot, driverController));
    robot.setDefaultCommand(new OperatorControls(robot, operatorController, buttonBoard));
    configureBindings();

    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto", autoChooser);
  }

  
  private void configureBindings() {
    //SmartDashboard.putData("Test Auto", new PathPlannerAuto("New Auto"));
    
    SmartDashboard.putData(new InstantCommand(()->{DriveSubsystem.resetGyro(0);}));
  }

 public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
    return null;
  }

  public void displayDebug() {
    drive.modules.debugSmartDashboard();
    robot.pivot.debugSmartDashboard();
    robot.debugSmartDashboard();
    // limelight.LimelightWhere();
    drive.modules.encoderVoltage();
  }
}