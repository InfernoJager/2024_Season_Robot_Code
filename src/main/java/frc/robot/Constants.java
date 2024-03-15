// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.motor.MotorInfo;
import frc.robot.swervemodule.SwerveModuleInfo;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double MODULE_ANGLE_KP = -0.00524;

  public static final int DRIVE_CONTROL_PORT = 0;
  public static final int OPERATOR_CONTROL_PORT = 1;
  public static final int BUTTON_BOARD_PORT = 2;

  //Center triangle is ID 15, Left triangle is ID 9
  public static final int BackRightDriveID = 5;
  public static final int BackRightSteerID = 6;
  public static final int BackLeftDriveID = 7;
  public static final int BackLeftSteerID = 8;
  public static final int FrontRightDriveID = 1;
  public static final int FrontRightSteerID = 2;
  public static final int FrontLeftDriveID = 3;
  public static final int FrontLeftSteerID = 4;
  public static final int MainPivotID = 9;
  public static final int SlavePivotID = 10;
  public static final int BeltID = 13;
  public static final int MainCannonID = 11;
  public static final int SlaveCannonID = 12;
  public static final int IntakeID = 14;
  public static final int ClimbID = 15;
  public static final double PivotOffset = 112.39;

  // Swerve
  public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(FrontRightDriveID, FrontRightSteerID, 3.343750, 360, 8.75, -1, -1);
  public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(FrontLeftDriveID, FrontLeftSteerID, 3.402344, 360, 330, -1, 1);
  public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(BackRightDriveID, BackRightSteerID, 3.406250, 360, 336.75, 1, -1);
  public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(BackLeftDriveID, BackLeftSteerID, 3.484375, 360, 174.75, 1, 1);

  // Robot
  public static final MotorInfo CANNON_MAIN = new MotorInfo(MainCannonID, 0, 0);
  public static final MotorInfo CANNON_SLAVE = new MotorInfo(SlaveCannonID, 0, 0);
  public static final MotorInfo PIVOT_MAIN = new MotorInfo(MainPivotID, 0, 112.39);
  public static final MotorInfo PIVOT_SLAVE = new MotorInfo(SlavePivotID, 0, 0);
  public static final MotorInfo FEEDER_BELT = new MotorInfo(BeltID, 0, 0);
  public static final MotorInfo INTAKE = new MotorInfo(IntakeID, 0, 0);
  public static final MotorInfo CLIMB_ARM = new MotorInfo(ClimbID, 0, 0);

  public static final class Swerve {
    public static final Translation2d flModuleOffset = new Translation2d(0.28, 0.28);
    public static final Translation2d frModuleOffset = new Translation2d(0.28, -0.28);
    public static final Translation2d blModuleOffset = new Translation2d(-0.28, 0.28);
    public static final Translation2d brModuleOffset = new Translation2d(-0.28, -0.28);

    public static final double maxModuleSpeed = 4.5; // M/S

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0), // Rotation constants 
      maxModuleSpeed, 
      flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }

}