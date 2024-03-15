package frc.robot.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.controller.PIDController;

public class Motor {
    
    public final CANSparkMax motor;
    public SparkAnalogSensor analogEncoder;
    public SparkAbsoluteEncoder absoluteEncoder;
    public RelativeEncoder inBuiltEncoder;
    private double orientation = 0.0;
    public final MotorInfo info;
    // private Timer time;
    // private double currentPosition;
    // private double positionChange;

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

    // public boolean getIsMaxSpeed(double maxMotorSpeed) {

    //     // Max motor speed is max rotations in a 0.2 second period. (rotations per second/20)

    //     boolean max;

    //     double timer = time.get();

    //     if (timer == 0) {
    //         currentPosition = inBuiltEncoder.getPosition();
    //         time.start();
    //     }

    //     if (timer >= 0) {
    //         positionChange = inBuiltEncoder.getPosition() - currentPosition;
    //         max = false;
    //     } else if (timer >= 0.2 && positionChange < maxMotorSpeed) {
    //         time.stop();
    //         time.reset();
    //         max = false;
    //     } else {
    //         max = true;
    //     }

    //     return max;

    // }

}
