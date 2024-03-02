package frc.robot.swervemodule;

import frc.robot.utils.VectorR;

public class SwerveModuleInfo {
    
    public final int DRIVE_ID;
    public final int TURN_ID;
    public final double ABS_ENCODER_MAX_VALUE;
    public final double ABS_ENCODER_VALUE_WHEN_STRAIGHT;
    public final double X;
    public final double Y;
    public final double MODULE_TANGENT_DEG;
    public final double MAX_ENCODER_VOLTAGE;

    public SwerveModuleInfo(int drive_motor_CAN_ID, int angle_motor_CAN_ID, double max_encoder_voltage, double abs_encoder_max_value, double abs_encoder_value_when_wheel_straight, double x, double y) {
        
        DRIVE_ID = drive_motor_CAN_ID;
        TURN_ID = angle_motor_CAN_ID;
        MAX_ENCODER_VOLTAGE = max_encoder_voltage;
        ABS_ENCODER_MAX_VALUE = abs_encoder_max_value;
        ABS_ENCODER_VALUE_WHEN_STRAIGHT = abs_encoder_value_when_wheel_straight;
        X = x;
        Y = y;
        MODULE_TANGENT_DEG = VectorR.fromCartesian(x, y).getAngle() - 90;

    }

}