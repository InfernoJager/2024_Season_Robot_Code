// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.motor.MotorInfo;
import frc.robot.swervemodule.SwerveModuleInfo;

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

  public static final int TestLeftID = 9;
  public static final int TestRightID = 15;
  public static final int BackRightDriveID = 5;
  public static final int BackRightSteerID = 6;
  public static final int BackLeftDriveID = 7;
  public static final int BackLeftSteerID = 8;
  public static final int FrontRightDriveID = 1;
  public static final int FrontRightSteerID = 2;
  public static final int FrontLeftDriveID = 3;
  public static final int FrontLeftSteerID = 4;
  public static final int LeftPivotID = 9;
  public static final int RightPivotID = 10;
  public static final int BeltID = 11;
  public static final int LeftCannonID = 9;
  public static final int RightCannonID = 15;
  public static final int IntakeID = 14;
  public static final int ClimbID = 15;

  // Swerve
  public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(FrontRightDriveID, FrontRightSteerID, 3.316, 360, 53.149125, 1, -1);
  public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(FrontLeftDriveID, FrontLeftSteerID, 3.339, 360, 262.675202, -1, -1);
  public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(BackRightDriveID, BackRightSteerID, 3.341, 360, 106.836277, 1, 1);
  public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(BackLeftDriveID, BackLeftSteerID, 3.316, 360, 126.046818, -1, 1);

  public static final MotorInfo CANNON_MAIN = new MotorInfo(LeftCannonID, 0, 0);
  public static final MotorInfo CANNON_SLAVE = new MotorInfo(RightCannonID, 0, 0);
  public static final MotorInfo PIVOT_MAIN = new MotorInfo(LeftPivotID, 0, 0);
  public static final MotorInfo PIVOT_SLAVE = new MotorInfo(RightPivotID, 0, 0);
  public static final MotorInfo FEEDER_BELT = new MotorInfo(BeltID, 0, 0);
  public static final MotorInfo INTAKE = new MotorInfo(IntakeID, 0, 0);
  public static final MotorInfo CLIMB_ARM = new MotorInfo(ClimbID, 0, 0);

}