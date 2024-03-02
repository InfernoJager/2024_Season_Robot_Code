package frc.robot.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;

public class Motor {
    
    public final CANSparkMax motor;
    public SparkAnalogSensor analogEncoder;
    public SparkAbsoluteEncoder absoluteEncoder;
    public RelativeEncoder inBuiltEncoder;
    private double orientation = 0.0;
    public final MotorInfo info;

    public Motor(MotorInfo info, boolean analog, boolean absolute) {

        this.info = info;
        this.motor = new CANSparkMax(info.ID, MotorType.kBrushless);
        this.inBuiltEncoder = motor.getEncoder();
        motor.setIdleMode(IdleMode.kBrake);

        if (analog) {
            this.analogEncoder = motor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        } 
        if (absolute) {
            this.absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        } 

    }

    public double getRawAngle(boolean degreesOrVoltage) {
        
        double rawAngle;

        if (degreesOrVoltage) {
            double degreePosition = analogEncoder.getPosition();
            
            rawAngle = degreePosition;
        } else {
            double degreesPerVolt = 360/info.MAX_ENCODER_VALUE;
            double encoderVoltage = analogEncoder.getVoltage();
            double voltsPosition = degreesPerVolt * encoderVoltage;

            rawAngle = voltsPosition;
        }

        return rawAngle;

    }

    public double getAngle() {
        
        double angle = getRawAngle(true) - info.REFERENCE_ANGLE;
        
        return angle;

    }

    public double getAbsoluteRawAngle() {

        double angle = absoluteEncoder.getPosition();

        return angle;

    }

    public double getAbsoluteAngle() {

        double angle = absoluteEncoder.getPosition() - info.REFERENCE_ANGLE;

        return angle;

    }

}
