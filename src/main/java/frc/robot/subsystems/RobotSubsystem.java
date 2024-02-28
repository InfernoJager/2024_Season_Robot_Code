package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.PairedMotors;
import frc.robot.motor.Motors;
import frc.robot.Constants;
import frc.robot.commands.DriverControls;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.math.controller.PIDController;
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
    private Timer feedTimer = new Timer();
    private boolean climbing = false;
    private boolean pivoting = false;
    public boolean readyToShoot = false;
    private double desiredAngle = 20;
    private double currentAngle;
    private double pivotspeed;
    private PIDController pivotpid;
    private double shootspeed;
    private double intakespeed;
    private double feedspeed;
    private robotState currentState = robotState.idle;
    private robotState queuedState = robotState.idle;
    public enum robotState{
        idle, shootPivot, speakerShootingPrep, speakerShootingFire, ampShooting, shootFinished, intakingPivot, intaking, climbingprep, climbing, readyToShoot;
    }
    private double pidCalcValue;
    private double pidSetValue;

    public RobotSubsystem() {
        
        this.cannon = new PairedMotors(Constants.CANNON_MAIN, Constants.CANNON_SLAVE, false, false);
        this.pivot = new PairedMotors(Constants.PIVOT_MAIN, Constants.PIVOT_SLAVE, false, true);
        this.pivot.mainMotor.absoluteEncoder.setInverted(true);
        // Test 0.02 for kd
        this.pivotpid = new PIDController(0.03, 0.000000001, 0.1);
        pivotpid.enableContinuousInput(0, 360);
        this.intake = new Motors(Constants.INTAKE, false, false);
        this.climb = new Motors(Constants.CLIMB_ARM, false, false);
        this.belt = new Motors(Constants.FEEDER_BELT, false, false);

        this.sensor = new AnalogInput(0);

    }

    public void resetStates() {

        queuedState = robotState.idle;
        currentState = robotState.idle;

    }

    public void resetMotors() {

        climb.Spin(0);
        pivot.Spin(0);
        belt.Spin(0);
        intake.Spin(0);
        cannon.Spin(0);


    }

    public void SetQueuedState(robotState state) {

        queuedState = state;

    }

    public void SetShootSpeed(double speed) {

        this.shootspeed = speed;

    }

    public void SetPivotSpeed(double speed) {

        this.pivotspeed = speed;

    }

    public void SetIntakeSpeed(double speed) {

        this.intakespeed = speed;

    }

    public void SetFeedSpeed(double speed) {

        this.feedspeed = speed;

    }

    public void Shoot() {
        
        if (currentState == robotState.readyToShoot && (queuedState == robotState.ampShooting || queuedState == robotState.speakerShootingPrep)) {
            currentState = robotState.shootPivot;
            shootTimer.reset();
            feedTimer.reset();
        }
        if (currentState == robotState.shootPivot) {
            PivotStart();
            Pivot();
        }
        if (currentState == robotState.shootPivot && GetNearDesiredAngle(GetDesiredAngle(), 5)) {
            currentState = queuedState;
        }
        if (currentState == robotState.ampShooting) {

            Feed(shootspeed);
            cannon.Spin(shootspeed);
            if (isNoteOut(sensor)) {
                shootTimer.start();
            }

        }
        if (currentState == robotState.speakerShootingPrep) {

            cannon.Spin(shootspeed);
            Feed(shootspeed);
            if (isNoteOut(sensor)) {
                //Feed(0);
                currentState = robotState.shootFinished;
            }

        }
        if (currentState == robotState.speakerShootingFire) {

            Feed(shootspeed);
            feedTimer.start();
            if (feedTimer.get() > 1 && isNoteOut(sensor)) {
                currentState = robotState.shootFinished;
            }

        }
        if (currentState == robotState.speakerShootingFire || currentState == robotState.ampShooting) {
            if (shootTimer.get() > 1) {
                currentState = robotState.shootFinished;
            }
        }
        if (currentState == robotState.shootFinished) {

            Feed(0);
            cannon.Spin(0);
            currentState = robotState.idle;
            SetQueuedState(robotState.idle);

        }

    }

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

    private boolean GetNearDesiredAngle(double targetAngle, double deadzone) {

        return (GetDesiredAngle() == targetAngle && GetDesiredAngle() > currentAngle - deadzone && GetDesiredAngle() < currentAngle + deadzone);

    }

    public void Pivot() {

        this.currentAngle = pivot.mainMotor.getAbsoluteRawAngle();

        if (pivoting) {
            boolean safezone = (currentAngle > 0 && currentAngle < 100);
            pidCalcValue = pivotpid.calculate(currentAngle, desiredAngle);

            if (safezone) {
                if (pivotpid.calculate(currentAngle, desiredAngle) > 0) {
                    pidSetValue = -Math.min(pivotpid.calculate(currentAngle, desiredAngle), pivotspeed);
                    pivot.Spin(pidSetValue);
                }
                if (pivotpid.calculate(currentAngle, desiredAngle) < 0) {
                    pidSetValue = -Math.max(pivotpid.calculate(currentAngle, desiredAngle), -pivotspeed);
                    pivot.Spin(pidSetValue);
                }
                readyToShoot = false;
            } else {
                pivot.Spin(0);
                pivoting = false;
                readyToShoot = true;
            }

        }

    }

    public void Intake() {
        
        if (currentState == robotState.idle && queuedState == robotState.intakingPivot) {
            currentState = robotState.intakingPivot;
        }
        if (currentState == robotState.intakingPivot) {
            PivotStart();
            Pivot();
        }
        if (GetNearDesiredAngle(1, 1) && currentState == robotState.intakingPivot) {
            SetPivotSpeed(0);
            currentState = robotState.intaking;
        }
        if (currentState == robotState.intaking) {
            intake.Spin(intakespeed);
            Feed(-intakespeed);
        }
        if (currentState == robotState.intaking) {
            if (isNoteIn(sensor)) {
                Feed(0);
                intake.Spin(0);
                resetStates();
                currentState = robotState.readyToShoot;
            }
        }

    }

    private void Feed(double speed) {

        belt.Spin(speed);

    }

    public void Climb(double speed) {

        

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
        SmartDashboard.putString("currentstate", currentState.toString());
        SmartDashboard.putString("queuedstate", queuedState.toString());
        SmartDashboard.putNumber("Test", climb.motor.motor.getEncoder().getPosition());
        SmartDashboard.putNumber("Pid", pidCalcValue);
        SmartDashboard.putNumber("pidval", pidSetValue);

    }
    
}
