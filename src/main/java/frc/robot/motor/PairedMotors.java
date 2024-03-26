package frc.robot.motor;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PairedMotors {
    
    public final Motor mainMotor;
    public final Motor slaveMotor;

    public PairedMotors(MotorInfo main, MotorInfo slave, boolean analog, boolean absolute) {
        
        
        this.mainMotor = new Motor(main, analog, absolute);
        this.slaveMotor = new Motor(slave, false, false);

        

    }

    public void Spin(double speed) {
        
        mainMotor.motor.set(speed);
        slaveMotor.motor.set(-speed);

    }
    
    public void SetRampRate(double rate) {

        mainMotor.motor.setClosedLoopRampRate(rate);
        slaveMotor.motor.setClosedLoopRampRate(rate);

    }

    public void debugSmartDashboard() {
        
        // SmartDashboard.putNumber("PivotConversin", mainMotor.absoluteEncoder.getPositionConversionFactor());

    }

}
