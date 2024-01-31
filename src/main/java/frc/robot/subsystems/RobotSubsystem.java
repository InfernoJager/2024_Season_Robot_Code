package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.EncodedMotor;
import frc.robot.motor.Motor;
import frc.robot.Constants;

public class RobotSubsystem extends SubsystemBase {
    
    public final Motor mainMotor;
    // public final EncodedMotor encodedMain;
    public final Motor slaveMotor;

    public RobotSubsystem() {
        
        mainMotor = new Motor(Constants.CANNON_MAIN);
        slaveMotor = new Motor(Constants.CANNON_SLAVE);

    }

    public void PairedSpin(double speed, double length) {
        
        mainMotor.motor.set(speed);
        slaveMotor.motor.set(-speed);
        
        Timer.delay(length);

        mainMotor.motor.set(0);
        slaveMotor.motor.set(0);

    }
    
}
