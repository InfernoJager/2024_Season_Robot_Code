package frc.robot.motor;

public class PairedMotors {
    
    public final Motor mainMotor;
    public final EncodedMotor encodedMotor;
    public final Motor slaveMotor;

    public PairedMotors(MotorInfo main, MotorInfo slave) {
        
        mainMotor = new Motor(main);
        encodedMotor = new EncodedMotor(main);
        slaveMotor = new Motor(slave);

    }

    public void Spin(double speed) {
        
        mainMotor.motor.set(speed);
        slaveMotor.motor.set(-speed);

    }

    public double getAngle() {
        
        double angle = encodedMotor.orientationEncoder.getPosition();
        
        return angle;

    }

}
