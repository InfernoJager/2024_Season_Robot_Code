package frc.robot.motor;

public class Motors {
    
    public final Motor motor;
    public final EncodedMotor encodedMotor;

    public Motors(MotorInfo motor) {

        this.motor = new Motor(motor);
        this.encodedMotor = new EncodedMotor(motor);

    }

    public void Spin(double speed) {

        motor.motor.set(speed);

    }

    public double getAngle() {
          
        double angle = encodedMotor.orientationEncoder.getPosition();
        
        return angle;

    }

}
