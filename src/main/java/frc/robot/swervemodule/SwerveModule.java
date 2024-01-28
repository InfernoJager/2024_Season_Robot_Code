// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervemodule;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

/** Add your docs here. */
public class SwerveModule {
  // HARDWARE

  public final CANSparkMax angleMotor;
  public final CANSparkMax driveMotor;
  public final SparkAnalogSensor orientationEncoder;

  // INFORMATION
  private final double defensiveAngleDeg;
  private double wheelOrientation = 0.0;
  public final SwerveModuleInfo info;

  public SwerveModule(SwerveModuleInfo info) {
    
    this.info = info;
    this.angleMotor = new CANSparkMax(info.TURN_ID, MotorType.kBrushless);
    this.driveMotor = new CANSparkMax(info.DRIVE_ID, MotorType.kBrushless);
    this.orientationEncoder = angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
    this.defensiveAngleDeg = VectorR.fromCartesian(info.X, info.Y).getAngle();
    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);
    
  }

  //RESET METHODS
  public void resetDriveEncoder() {
    // driveMotor.setSelectedSensorPosition(0);
  }

  public double getWheelAngle() {
    
    double encoderVoltage;
    double degreesPerVolt;

    degreesPerVolt = 360/info.MAX_ENCODER_VOLTAGE;
    encoderVoltage = orientationEncoder.getVoltage();
    
    return encoderVoltage * degreesPerVolt;

  }

  public double getAngle() {
    return this.getWheelAngle() - info.ABS_ENCODER_VALUE_WHEN_STRAIGHT;
  }

  /*
   * positive (+) = left turn CCW
   * negative (-) = right turn CW
   */
  public double getWheelOrientationDegrees() {
    return wheelOrientation - info.ABS_ENCODER_VALUE_WHEN_STRAIGHT;
  }

  // MODULE SPEEDS CALCULATIONS
  private VectorR desired = new VectorR();
  private boolean reversed = false;

  private void reverse() {
    reversed = !reversed;
  }

  private double desiredSpeed() {
    if (reversed)
      return desired.getTerminalMagnitude();
    else
      return desired.getMagnitude();
  }

  private double desiredAngle() {
    if (reversed)
      return desired.getTerminalAngle();
    else
      return desired.getAngle();
  }

  /*
   * UPDATE OR STOP METHODS MUST BE CALLED PERIODICALLY 
   * speed 0 min - 1 max, turns module drive wheel
   * angle degrees follows coordinate plane standards, sets module wheel to angle
   */
  public void update(double speed, double angleDegrees) {
    wheelOrientation = getWheelAngle();

    desired.setFromPolar(speed, angleDegrees);

    if (Math.abs(MathR.getDistanceToAngle(getWheelOrientationDegrees(), desiredAngle())) > 90d)
      reverse();

    double speed_power = MathR.limit(desiredSpeed(), -1, 1);
    double angle_power = MathR
        .limit(Constants.MODULE_ANGLE_KP * MathR.getDistanceToAngle(getWheelOrientationDegrees(), desiredAngle()), -1, 1);
   
    if (Math.abs(angle_power) < 0.05) {
      angle_power = 0;
    }
    if (Math.abs(speed_power) < 0.01) {
      speed_power = 0;
    }
    System.out.println(speed_power + " " + angle_power);
    driveMotor.set(speed_power); 
    angleMotor.set(angle_power);

    
  }

  public void stop() {
    angleMotor.set(0);
    driveMotor.set(0);
  }

  public void stopDefensively() {
    update(0.0000000000000000000000000000000000000001,  defensiveAngleDeg);
  }
}