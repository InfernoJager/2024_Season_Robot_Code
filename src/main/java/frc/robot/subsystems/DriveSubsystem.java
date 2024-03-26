// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swervemodule.SwerveModule;
import frc.robot.swervemodule.SwerveModules;
import frc.robot.utils.VectorR;

public class DriveSubsystem extends SubsystemBase {

  // COMPONENTS
  public final SwerveModules modules;
  private static AHRS gyro;
  public XboxController driverController;

  // OTHER
  private boolean defensiveMode = true;
  private static double yawOffsetDegrees = 0;
  private double desiredYaw;
  private double currentYaw;

  public DriveSubsystem() {
    modules = new SwerveModules(
        new SwerveModule(Constants.FRONT_RIGHT), new SwerveModule(Constants.FRONT_LEFT),
        new SwerveModule(Constants.BACK_RIGHT), new SwerveModule(Constants.BACK_LEFT));

    gyro = new AHRS();
    gyro.reset();
  }

  /*
   * SYSTEM STANDARD FOLLOWS COORDINATE PLANE STANDARD
   * positive (+) = forwards/left/left turn CCW
   * negative (-) = backwards/right/right turn CW
   * velocity magnitude (0-1) 1:fastest 0:stopped
   * turn (0-1)
   * NOTE: the speed of any wheel can reach a maximum of turn + |velocity|
   */

  public double GetGyro() {

    double yaw = gyro.getYaw() + 180;

    return yaw;

  }
  
  public void move(VectorR directionalSpeed, double turnSpeed, boolean aPressed, boolean startPressed, boolean turning) {
    
    double speedMultiplier;
    

    if (startPressed && aPressed) {
      speedMultiplier = 1;
    } else if (aPressed) {
      speedMultiplier = 0.5;
    } else if (startPressed) {
      speedMultiplier = 2;
    } else {
      speedMultiplier = 1;
    }

    if (DriverStation.isAutonomous()) {
      currentYaw = gyro.getYaw() + 180;

      if (turning) {
        desiredYaw = currentYaw;
      } else if (desiredYaw == 0) {
        desiredYaw = currentYaw;
      }

      if (!turning && currentYaw > desiredYaw - 0.25) {
        turnSpeed = -0.0475;
      } else if (!turning && currentYaw < desiredYaw + 0.25) {
        turnSpeed = 0.0475;
      } else if (!turning) {
        turnSpeed = 0;
      }
    }

    VectorR directionalPull = directionalSpeed.clone();
    directionalPull.rotate(getYawDegrees() + 90);

    for (SwerveModule module : modules) {

      VectorR rotationalPull = VectorR.fromPolar(turnSpeed, module.info.MODULE_TANGENT_DEG);
      VectorR wheelPull = VectorR.addVectors(directionalPull, rotationalPull);

      module.update(wheelPull.getMagnitude() * speedMultiplier, wheelPull.getAngle());

    }

  }
  
  public void stop() {

    for (SwerveModule module : modules) {
      
      module.stop();

    }

  }

  public void setDefensiveMode(boolean activated) {
    defensiveMode = activated;
  }
  public boolean getDefensiveMode() {
    return defensiveMode;
  }

  /*
   * positive (+) = left turn CCW
   * negative (-) = right turn CW
   */
  public static double getYawDegrees() {
    return -1 * gyro.getYaw() + yawOffsetDegrees;
  }

  public static void resetGyro(double yawDegrees) {
    gyro.reset();
    yawOffsetDegrees = yawDegrees;
  }
  
  public void resetDriveEncoders() {
    for (var mod : modules)
      mod.resetDriveEncoder();
  }

  public double getRawMagX(){
    return gyro.getRawMagX();
  }

  public double getRawMagY(){
    return gyro.getRawMagY();
  }
}