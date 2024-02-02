package frc.robot.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class Motor {
    
    public final CANSparkMax motor;
    public SparkAnalogSensor orientationEncoder;
    private double orientation = 0.0;
    public final MotorInfo info;

    public Motor(MotorInfo info, boolean encoded) {

        this.info = info;
        this.motor = new CANSparkMax(info.ID, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);

        if (encoded) {
            this.orientationEncoder = motor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        }

    }

    public double getRawAngle(boolean degreesOrVoltage) {
        
        double rawAngle;

        if (degreesOrVoltage) {
            double degreePosition = orientationEncoder.getPosition();
            
            rawAngle = degreePosition;
        } else {
            double degreesPerVolt = 360/info.MAX_ENCODER_VALUE;
            double encoderVoltage = orientationEncoder.getVoltage();
            double voltsPosition = degreesPerVolt * encoderVoltage;

            rawAngle = voltsPosition;
        }

        return rawAngle;

    }

    public double getAngle() {
        
        double angle = getRawAngle(true) - info.REFERENCE_ANGLE;
        
        return angle;

    }

}
