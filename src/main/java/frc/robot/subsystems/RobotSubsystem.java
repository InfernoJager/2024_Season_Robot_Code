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
    private Timer shootTimer = new Timer();
    private Boolean shooting = false;
    private Timer intakeTimer = new Timer();
    private Boolean intaking = false;
    private Timer climbTimer = new Timer();
    private boolean climbing = false;
    private boolean pivoting = false;
    public boolean readyToShoot = false;

    public RobotSubsystem() {
        
        this.cannon = new PairedMotors(Constants.CANNON_MAIN, Constants.CANNON_SLAVE, false, false);
        this.pivot = new PairedMotors(Constants.PIVOT_MAIN, Constants.PIVOT_SLAVE, false, true);
        this.intake = new Motors(Constants.INTAKE, false, false);
        this.climb = new Motors(Constants.CLIMB_ARM, false, false);
        this.belt = new Motors(Constants.FEEDER_BELT, false, false);

    }

    public void Shoot(double speed) {

        if (!shooting) {
            shooting = true;

            cannon.Spin(speed);

            shootTimer.reset();
            shootTimer.start();
        }

    }

    public void ShootStop(double length) {

        if (shootTimer.get() > length) {
            shooting = false;
            readyToShoot = false;
            
            cannon.Spin(0);
        }

    }

    public void PivotStart() {

        if (!pivoting) {
            pivoting = true;
        }

    }

    public void Pivot(double currentAngle, double desiredAngle, double speed) {

        if (pivoting) {
            boolean safezone = (desiredAngle > 0 || desiredAngle < 119);
        
            if (Math.abs(currentAngle - desiredAngle) > 1 && safezone) {
                pivot.Spin(speed);
                readyToShoot = false;
            } else {
                pivot.Spin(0);
                pivoting = false;
                readyToShoot = true;
            }
        }

    }

    public void Intake(double speed) {

        if (!intaking) {
            intaking = true;
            
            intake.Spin(speed);

            intakeTimer.reset();
            intakeTimer.start();
        }

    }

    public void IntakeStop(double length) {

        if (intakeTimer.get() > length) {
            intaking = false;

            intake.Spin(0);
        }

    }

    public void Feed(double speed, boolean note) {

        if (note) {
            belt.Spin(0);
        } else {
            belt.Spin(speed);
        }

    }

    public void Climb(double speed) {

        if (!climbing) {
            climbing = true;
            
            climb.Spin(speed);

            climbTimer.reset();
            climbTimer.start();
        }

    }

    public void ClimbStop(double length) {

        if (climbTimer.get() > length) {
            climbing = false;

            climb.Spin(0);
        }

    }
    
}
