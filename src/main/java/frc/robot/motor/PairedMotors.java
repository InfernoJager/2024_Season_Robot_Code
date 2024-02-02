package frc.robot.motor;

public class PairedMotors {
    
    public final Motor mainMotor;
    public final Motor slaveMotor;

    public PairedMotors(MotorInfo main, MotorInfo slave, boolean encoded) {
        
        this.mainMotor = new Motor(main, encoded);
        this.slaveMotor = new Motor(slave, false);

    }

    public void Spin(double speed) {
        
        mainMotor.motor.set(speed);
        slaveMotor.motor.set(-speed);

    }

}
