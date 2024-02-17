package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.PairedMotors;
import frc.robot.motor.Motors;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.AnalogInput;

public class RobotSubsystem extends SubsystemBase {
    
    public final PairedMotors cannon;
    public final PairedMotors pivot;
    public final Motors intake;
    public final Motors climb;
    public final Motors belt;
    private final AnalogInput sensor;
    private Timer shootTimer = new Timer();
    private Timer climbTimer = new Timer();
    private boolean climbing = false;
    private boolean pivoting = false;
    public boolean readyToShoot = false;
    private double desiredAngle = 20;
    private double currentAngle;
    private robotState currentState = robotState.idle;
    private robotState queuedState = robotState.idle;
    //private sideState currentSideState = sideState.idle;
    public enum robotState{
        idle, speakerShooting, ampShooting, intakingPivot, intaking, climbing;
    }
    // public enum sideState{
    //     idle, pivoting, feeding;
    // }

    public RobotSubsystem() {
        
        this.cannon = new PairedMotors(Constants.CANNON_MAIN, Constants.CANNON_SLAVE, false, false);
        this.pivot = new PairedMotors(Constants.PIVOT_MAIN, Constants.PIVOT_SLAVE, false, true);
        this.intake = new Motors(Constants.INTAKE, false, false);
        this.climb = new Motors(Constants.CLIMB_ARM, false, false);
        this.belt = new Motors(Constants.FEEDER_BELT, false, false);

        this.sensor = new AnalogInput(0);

    }

    public void SetQueuedState(robotState state) {

        queuedState = state;

    }

    private void QueuedState() {

        if (currentState == robotState.idle) {
            currentState = queuedState;
        }

    }

    public void Shoot(double speed, robotState state) {

        if (currentState != robotState.speakerShooting && currentState != robotState.ampShooting && GetNearDesiredAngle(0, 1)) {
            currentState = state;

            cannon.Spin(speed);

            shootTimer.reset();
            shootTimer.start();
        }

        if (currentState == robotState.speakerShooting) {
            Feed(-0.05);

            Feed(0.05);
        }

    }

    public void ShootStop(double length) {

        if (shootTimer.get() > length) {
            currentState = robotState.idle;
            readyToShoot = false;

            shootTimer.stop();
            
            cannon.Spin(0);
        }

    }

    // public boolean ShootFinished(double length) {

    //     return (shootTimer.get() > length);

    // }

    public void PivotStart() {

        if (!pivoting) {
            pivoting = true;
        }

    }

    public void SetDesiredAngle(double desiredAngle) {

        this.desiredAngle = desiredAngle;

    }

    public double GetDesiredAngle() {

        return desiredAngle;

    }

    public boolean GetNearDesiredAngle(double targetAngle, double deadzone) {

        return (GetDesiredAngle() == targetAngle && GetDesiredAngle() > currentAngle - deadzone && GetDesiredAngle() < currentAngle + deadzone);

    }

    public void Pivot(double currentAngle, double speed) {

        if (pivoting) {
            boolean safezone = (desiredAngle > 0 || desiredAngle < 119);
        
            if (Math.abs(currentAngle - desiredAngle) > 5 && safezone) {
                pivot.Spin(speed);
                readyToShoot = false;
            } else {
                pivot.Spin(0);
                pivoting = false;
                readyToShoot = true;
            }

            this.currentAngle = currentAngle;

        }

    }

    public void Intake(double speed) {
        
        QueuedState();
        
        if (currentState == robotState.intakingPivot) {
            PivotStart();
            Pivot(pivot.mainMotor.getAbsoluteRawAngle(), 0.05);
        }
        if (GetNearDesiredAngle(20, 5) && currentState == robotState.intakingPivot || currentState == robotState.intaking) {
            intake.Spin(speed);
            currentState = robotState.intaking;
        }
        if (currentState == robotState.intaking) {
            Feed(0.05);
        }
        if (isNoteIn(sensor)) {
            Feed(0);
            intake.Spin(0);
            SetQueuedState(robotState.idle);
            currentState = robotState.idle;
        }

        System.out.println(currentState);

    }

    public void Feed(double speed) {

        belt.Spin(speed);

    }

    // public void FeedStop(boolean note) {

    //     if (note) {
    //         belt.Spin(0);
    //     }
        
    // }

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

    private boolean isNoteIn(AnalogInput sensor) {

        boolean isReady = false;

        isReady = sensor.getVoltage() < 0.01;

        return isReady;

    }

    private boolean isNoteOut(AnalogInput sensor) {

        boolean isReady = true;

        isReady = sensor.getVoltage() > 0.1;

        return isReady;

    }

    public void debugSmartDashboard() {

        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        SmartDashboard.putNumber("sensor", sensor.getVoltage());

    }
    
}
