package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.PairedMotors;
import frc.robot.Constants;

public class RobotSubsystem extends SubsystemBase {
    
    public final PairedMotors cannon;
    public final PairedMotors pivot;

    public RobotSubsystem() {
        
        this.cannon = new PairedMotors(Constants.CANNON_MAIN, Constants.CANNON_SLAVE, true);
        this.pivot = new PairedMotors(Constants.PIVOT_MAIN, Constants.PIVOT_SLAVE, true);

    }

    public void Shoot(double speed, double length) {

        cannon.Spin(speed);

        Timer.delay(length);

        cannon.Spin(0);

    }

    public void Pivot(double currentAngle, double desiredAngle, double speed, double minAngle, double maxAngle) {

        boolean safezone = (desiredAngle > minAngle || desiredAngle < maxAngle);
        
        if (Math.abs(currentAngle - desiredAngle) > 1 && safezone) {
            pivot.Spin(speed);
        } else {
            pivot.Spin(0);
        }

    }
    
}
