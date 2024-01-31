package frc.robot.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
// import frc.robot.Constants;
// import frc.robot.utils.MathR;
// import frc.robot.utils.VectorR;

public class Motor {
    
    public final CANSparkMax motor;
    public final MotorInfo info;

    public Motor(MotorInfo info) {

        this.info = info;
        this.motor = new CANSparkMax(info.ID, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);

    }

}