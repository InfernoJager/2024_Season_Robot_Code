// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double MODULE_ANGLE_KP = 0.00524;

  public static final int DRIVE_CONTROL_PORT = 0;

  public static final int BackLeftTriangleID = 9;
  public static final int BackMiddleTriangleID = 15;
  public static final int BackRightDriveID = 5;
  public static final int BackRightSteerID = 6;
  public static final int BackLeftDriveID = 7;
  public static final int BackLeftSteerID = 8;
  public static final int FrontRightDriveID = 1;
  public static final int FrontRightSteerID = 2;
  public static final int FrontLeftDriveID = 3;
  public static final int FrontLeftSteerID = 4;

  // Swerve
  public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(FrontRightDriveID, FrontRightSteerID, 360, 0, 1, -1);
  public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(FrontLeftDriveID, FrontLeftSteerID, 360, 0, 1, 1);
  public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(BackRightDriveID, BackRightSteerID, 360, 0, -1, -1);
  public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(BackLeftDriveID, BackLeftSteerID, 360, 0, -1, 1);

  
}