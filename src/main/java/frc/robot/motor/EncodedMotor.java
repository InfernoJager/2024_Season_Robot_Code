package frc.robot.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class EncodedMotor {
    
    public final CANSparkMax motor;
    public final SparkAnalogSensor orientationEncoder;
    private double orientation = 0.0;
    public final MotorInfo info;

    public EncodedMotor(MotorInfo info) {

        this.info = info;
        this.motor = new CANSparkMax(info.ID, MotorType.kBrushless);
        this.orientationEncoder = motor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        motor.setIdleMode(IdleMode.kBrake);

    }

}
