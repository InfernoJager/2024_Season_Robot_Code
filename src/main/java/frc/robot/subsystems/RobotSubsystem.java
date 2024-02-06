package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.PairedMotors;
import frc.robot.motor.Motors;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTag;

public class RobotSubsystem extends SubsystemBase {
    
    public final PairedMotors cannon;
    public final PairedMotors pivot;
    public final Motors intake;
    public final Motors climb;
    public final Motors belt;

    public RobotSubsystem() {
        
        this.cannon = new PairedMotors(Constants.CANNON_MAIN, Constants.CANNON_SLAVE, false, false);
        this.pivot = new PairedMotors(Constants.PIVOT_MAIN, Constants.PIVOT_SLAVE, false, true);
        this.intake = new Motors(Constants.INTAKE, false, false);
        this.climb = new Motors(Constants.CLIMB_ARM, false, false);
        this.belt = new Motors(Constants.FEEDER_BELT, false, false);

    }

    public void Shoot(double speed, double length) {

        cannon.Spin(speed);

        Timer.delay(length);

        cannon.Spin(0);

    }

    public void Pivot(double currentAngle, double desiredAngle, double speed) {

        boolean safezone = (desiredAngle > 20 || desiredAngle < 119);
        
        if (Math.abs(currentAngle - desiredAngle) > 1 && safezone) {
            pivot.Spin(speed);
        } else {
            pivot.Spin(0);
        }

    }

    public void Intake(double speed, double length) {

        intake.Spin(speed);

        Timer.delay(length);

        intake.Spin(0);

    }

    public void Feed(double speed, boolean note) {

        if (note) {
            belt.Spin(0);
        } else {
            belt.Spin(speed);
            Timer.delay(3);
            belt.Spin(0);
        }

    }

    public void climb(double speed, double length) {

        climb.Spin(speed);

        Timer.delay(length);

        climb.Spin(0);

    }
    
}
