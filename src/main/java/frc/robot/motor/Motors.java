package frc.robot.motor;

public class Motors {
    
    public final Motor motor;

    public Motors(MotorInfo motor, boolean analog, boolean absolute) {

        this.motor = new Motor(motor, analog, absolute);


    }

    public void Spin(double speed) {

        motor.motor.set(speed);

    }

}
