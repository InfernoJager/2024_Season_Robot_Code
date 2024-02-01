package frc.robot.motor;

public class MotorInfo {
    
    public final int ID;
    public final double MAX_ENCODER_VALUE;
    public final double REFERENCE_ANGLE;

    public MotorInfo(int MOTOR_CAN_ID, double MAX_ENCODER_VALUE, double REFERENCE_ANGLE) {
        
        this.ID = MOTOR_CAN_ID;
        this.MAX_ENCODER_VALUE = MAX_ENCODER_VALUE;
        this.REFERENCE_ANGLE = REFERENCE_ANGLE;
        
    }

}
