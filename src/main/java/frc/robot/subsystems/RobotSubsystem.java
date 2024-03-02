package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
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
        idle, shootPivot, speakerShooting, ampShooting, ampShootingFinal, shooting, shootFinished, intakingPivot, intaking, climbingprep, climbing, readyToShoot, noteRetractingStart, noteRetracting;
    }
    private double pidCalcValue;
    private double pidSetValue;
    private double pidFinalValue;
    private double wantedFeed;
    private double wantedShoot;

    public RobotSubsystem() {
        
        this.cannon = new PairedMotors(Constants.CANNON_MAIN, Constants.CANNON_SLAVE, false, false);
        this.pivot = new PairedMotors(Constants.PIVOT_MAIN, Constants.PIVOT_SLAVE, false, true);
        this.pivot.mainMotor.absoluteEncoder.setInverted(true);
        // Test 0.02 for kd
        this.pivotpid = new PIDController(0.02, 0, 0.001);
        pivotpid.enableContinuousInput(0, 360);
        pivotpid.setTolerance(1);
        this.intake = new Motors(Constants.INTAKE, false, false);
        this.climb = new Motors(Constants.CLIMB_ARM, false, false);
        this.belt = new Motors(Constants.FEEDER_BELT, false, false);

        this.sensor = new AnalogInput(0);

    }

    public void resetStates() {

        queuedState = robotState.idle;
        currentState = robotState.idle;

    }

    public void revertStates() {

        if (queuedState == robotState.intakingPivot) {
            queuedState = robotState.idle;
            currentState = robotState.idle;
        }
        if (queuedState == robotState.ampShooting || queuedState == robotState.speakerShooting) {
            queuedState = robotState.readyToShoot;
            currentState = robotState.readyToShoot;
        }

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

    public void NoteBack() {

        double currentFeed = Math.abs(belt.motor.inBuiltEncoder.getPosition());

        if (currentState == robotState.noteRetractingStart) {
            Feed(0.1);
        }
        if (isNoteOut(sensor) && currentState == robotState.noteRetractingStart) {
            wantedFeed = currentFeed - 1.75;
            currentState = robotState.noteRetracting;
        }
        if (currentState == robotState.noteRetracting && currentFeed <= wantedFeed) {
            Feed(0);
            currentState = robotState.readyToShoot;
        }

    }

    public void Shoot() {
        
        double target = 0;
        double currentShoot = cannon.mainMotor.inBuiltEncoder.getPosition();

        if (currentState == robotState.readyToShoot && (queuedState == robotState.ampShooting || queuedState == robotState.speakerShooting)) {
            currentState = robotState.shootPivot;
        }
        if (currentState == robotState.shootPivot) {
            PivotStart();
            Pivot();
        }
        if (queuedState == robotState.ampShooting) {
            target = 117;
        }
        if (queuedState == robotState.speakerShooting) {
            target = 61;
        }
        if (currentState == robotState.shootPivot && GetNearDesiredAngle(target, 0.75) && Math.abs(pidFinalValue) < 0.05) {
            currentState = queuedState;
        }
        if (currentState == robotState.ampShooting) {

            Feed(-0.5);
            cannon.Spin(shootspeed);

        }
        if (currentState == robotState.speakerShooting) {

            cannon.Spin(shootspeed);
            Feed(shootspeed);

        }
        if (currentState == robotState.speakerShooting || currentState == robotState.ampShooting) {
            
            if (isNoteIn(sensor)) {
                currentState = robotState.shooting;
            }
            
        }
        if (currentState == robotState.shooting) {

            if (isNoteOut(sensor) && queuedState == robotState.speakerShooting) {
                currentState = robotState.shootFinished;
            }
            if (isNoteOut(sensor) && queuedState == robotState.ampShooting) {
                wantedShoot = currentShoot - 5;
                currentState = robotState.ampShootingFinal;
            }

        }
        if (currentState == robotState.ampShootingFinal && currentShoot <= wantedShoot) {

            currentState = robotState.shootFinished;

        }
        if (currentState == robotState.shootFinished) {

            Feed(0);
            cannon.Spin(0);
            SetDesiredAngle(33);
            PivotStart();
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

        double speedMultiplier;
        double angleOffset = 20;
        this.currentAngle = pivot.mainMotor.getAbsoluteRawAngle() + angleOffset;
        if (this.currentAngle > 360) {
            currentAngle = this.currentAngle - 360;
        }

        if (currentAngle < 50 && desiredAngle == 33 && pidFinalValue > 0.1) {
            speedMultiplier = 0.1;
        } else if (currentAngle < 35 && desiredAngle == 21 && pidFinalValue > 0.1) {
            speedMultiplier = 0.1;
        } else if (currentAngle > 105 && desiredAngle == 118 && pidFinalValue < -0.1) {
            speedMultiplier = 0.1;
        } else {
            speedMultiplier = 1;
        }

        if (pivoting) {
            boolean safezone = (currentAngle > 20 && currentAngle < 120);
            pidCalcValue = pivotpid.calculate(currentAngle, desiredAngle);
            if (currentAngle < 20) {
                pivot.Spin(-0.1);
            }
            if (currentAngle > 120) {
                pivot.Spin(0.1);
            }
            if (safezone) {
                if (pidCalcValue > 0) {
                    pidSetValue = Math.min(pidCalcValue, -pivotspeed);  
                }
                if (pidCalcValue < 0) {
                    pidSetValue = Math.max(pidCalcValue, pivotspeed);
                }
                if (desiredAngle >= currentAngle) {
                    pidFinalValue = -Math.abs(pidSetValue);
                } 
                if (desiredAngle <= currentAngle) {
                    pidFinalValue = Math.abs(pidSetValue);
                }
                pivot.Spin(pidFinalValue * speedMultiplier);
                readyToShoot = false;
            }

        } else {
            pivoting = false;
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
        if (GetNearDesiredAngle(21, 1) && currentState == robotState.intakingPivot) {
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
                SetDesiredAngle(33);
                PivotStart();
                queuedState = robotState.readyToShoot;
                currentState = robotState.noteRetractingStart;
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
        SmartDashboard.putNumber("BeltHall", belt.motor.inBuiltEncoder.getPosition());

    }
    
}
