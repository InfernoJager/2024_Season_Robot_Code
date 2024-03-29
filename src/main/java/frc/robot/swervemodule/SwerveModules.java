// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervemodule;

import java.util.HashMap;
import java.util.Iterator;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Makes it easier to access swerve modules */
public class SwerveModules implements Iterable<SwerveModule> {

  // MODULES
  public final SwerveModule frontRight;
  public final SwerveModule frontLeft;
  public final SwerveModule backRight;
  public final SwerveModule backLeft;

  private final HashMap<ModuleLocation, SwerveModule> modules = new HashMap<>();

  public SwerveModules(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backRight, SwerveModule backLeft) {
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    this.backRight = backRight;
    this.backLeft = backLeft;

    modules.put(ModuleLocation.FRONT_RIGHT, frontRight);
    modules.put(ModuleLocation.FRONT_LEFT, frontLeft);
    modules.put(ModuleLocation.BACK_RIGHT, backRight);
    modules.put(ModuleLocation.BACK_LEFT, backLeft);
  }

  // get a module based on an enum value
  public SwerveModule get(ModuleLocation module) {
    return modules.get(module);
  }

  public enum ModuleLocation {
    FRONT_RIGHT,
    FRONT_LEFT,
    BACK_RIGHT,
    BACK_LEFT
  }

  // get a list of the modules for looping through (iterating)
  @Override
  public Iterator<SwerveModule> iterator() {
    return modules.values().iterator();
  }

  public void debugSmartDashboard() {

    // Shuffleboard.getTab("Programming Data");
    SmartDashboard.putNumber("FRAngle", frontRight.getWheelAngle());
    SmartDashboard.putNumber("FRPower", frontRight.angleMotor.get());
    SmartDashboard.putNumber("FLAngle", frontLeft.getWheelAngle());
    SmartDashboard.putNumber("FLPower", frontLeft.angleMotor.get());
    SmartDashboard.putNumber("BRAngle", backRight.getWheelAngle());
    SmartDashboard.putNumber("BRPower", backRight.angleMotor.get());
    SmartDashboard.putNumber("BLAngle", backLeft.getWheelAngle());
    SmartDashboard.putNumber("BLPower", backLeft.angleMotor.get());

  } 

  public void encoderVoltage() {
    SmartDashboard.putNumber("FR", frontRight.orientationEncoder.getVoltage());
    SmartDashboard.putNumber("FL", frontLeft.orientationEncoder.getVoltage());
    SmartDashboard.putNumber("BR", backRight.orientationEncoder.getVoltage());
    SmartDashboard.putNumber("BL", backLeft.orientationEncoder.getVoltage());
  }

  public void SwerveStatus() {
    
    SmartDashboard.putNumber("CAN1Temp", frontLeft.driveMotor.getMotorTemperature());
    SmartDashboard.putNumber("CAN1Volt", frontLeft.driveMotor.getBusVoltage());
    SmartDashboard.putNumber("CAN2Temp", frontLeft.angleMotor.getMotorTemperature());
    SmartDashboard.putNumber("CAN2Volt", frontLeft.angleMotor.getBusVoltage());
    SmartDashboard.putNumber("CAN3Temp", frontRight.driveMotor.getMotorTemperature());
    SmartDashboard.putNumber("CAN3Volt", frontRight.driveMotor.getBusVoltage());
    SmartDashboard.putNumber("CAN4Temp", frontRight.angleMotor.getMotorTemperature());
    SmartDashboard.putNumber("CAN4Volt", frontRight.angleMotor.getBusVoltage());
    SmartDashboard.putNumber("CAN5Temp", backLeft.driveMotor.getMotorTemperature());
    SmartDashboard.putNumber("CAN5Volt", backLeft.driveMotor.getBusVoltage());
    SmartDashboard.putNumber("CAN6Temp", backLeft.angleMotor.getMotorTemperature());
    SmartDashboard.putNumber("CAN6Volt", backLeft.angleMotor.getBusVoltage());
    SmartDashboard.putNumber("CAN7Temp", backRight.driveMotor.getMotorTemperature());
    SmartDashboard.putNumber("CAN7Volt", backRight.driveMotor.getBusVoltage());
    SmartDashboard.putNumber("CAN8Temp", backRight.angleMotor.getMotorTemperature());
    SmartDashboard.putNumber("CAN8Volt", backRight.angleMotor.getBusVoltage());
    
  }

}