package frc.robot.motor;

public class Motors {
    
    public final Motor motor;

    public Motors(MotorInfo motor, boolean encoded) {

        this.motor = new Motor(motor, encoded);

    }

    public void Spin(double speed) {

        motor.motor.set(speed);

    }

}
