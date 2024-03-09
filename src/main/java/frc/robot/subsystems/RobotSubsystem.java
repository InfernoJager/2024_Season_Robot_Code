package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
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
    private double climbSpeed = 0;
    private double climbRevs;
    private boolean climbSafe = false;
    private double modifiedPivot;
    private boolean pivotDone;
    private boolean climbDone;
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
        idle, shootPivot, speakerShooting, ampShooting, ampShootingFinal, shooting, shootFinished, intakingPivot, intaking, climbingprep, readyToClimb, climbing, readyToShoot, noteRetractingStart, noteRetracting, matchFinish;
    }
    private double pidCalcValue;
    private double pidSetValue;
    private double pidFinalValue;
    private double wantedFeed;
    private double wantedShoot;
    private Servo cannonLockLeft;
    private Servo cannonLockRight;    

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
        this.cannonLockRight = new Servo(1);
        this.cannonLockLeft = new Servo(2);

    }

    public void resetStates() {

        queuedState = robotState.idle;
        currentState = robotState.idle;

    }

    public void revertStates() {

        if (queuedState == robotState.intakingPivot || queuedState == robotState.readyToShoot) {
            queuedState = robotState.idle;
            currentState = robotState.idle;
        }
        if (queuedState == robotState.ampShooting || queuedState == robotState.speakerShooting || queuedState == robotState.idle) {
            queuedState = robotState.readyToShoot;
            currentState = robotState.readyToShoot;
        }
        if (currentState == robotState.matchFinish) {
            SetDesriedClimb(13.25);
            queuedState = robotState.climbingprep;
            currentState = robotState.idle;
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

    public void SetClimbSpeed(double speed) {

        this.climbSpeed = speed;

    }

    public void SetDesriedClimb(double length) {

        // Length is in inches

        this.climbRevs = length * 8;
    
    }


    public robotState GetRobotCurrentState(){
        return currentState;
    }

    public void SetRobotCurrentState(robotState state){
        currentState = state;
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
            target = 117.5;
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
        } else if (currentAngle < 24 && desiredAngle == 21 && pidFinalValue < 0.03) {
            speedMultiplier = 5;
        } else if (currentAngle < 35 && desiredAngle == 21 && pidFinalValue > 0.1) {
            speedMultiplier = 0.1;
        } else if (currentAngle > 105 && desiredAngle == 117.5 && pidFinalValue < -0.1) {
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

    public void ClimbStart() {

        if (!climbing) {
            climbing = true;
        }

    }

    private boolean IsArmExtened(double revolutions) {
        boolean answer;
        // double maxValue;
        double minValue;
        // maxValue = climbRevs + 1;
        minValue = climbRevs - 1;
        answer =  (revolutions > minValue);
        return answer;
    }

    private boolean IsArmRetracted(double revolutions){
        boolean answer;
        double maxValue;
        // double minValue;
        maxValue = climbRevs + 1;
        // minValue = climbRevs - 1;
        answer =  (revolutions < maxValue);
        return answer;
    }

    private boolean IsArmShorterThanLimit(double revolutions){
        boolean answer;
        double checkValue;
        checkValue = 4;
        answer = revolutions < checkValue;
        return answer;
    }

    private boolean IsArmLongerThanLimit(double revolutions){
        boolean answer;
        double checkValue;
        checkValue = 108;
        answer = revolutions > checkValue;
        return answer;
    }

    private void ExtendArm(double power, boolean rememberPower){
        //negative power value extends arm
        double pwr;
        pwr = -Math.abs(power);
		if (rememberPower) {
			previousArmPower = pwr;
		}
        climb.Spin(pwr);
    }

    private void RetractArm(double power, boolean rememberPower){
        //positive power value retracts arm
        double pwr;
        pwr = Math.abs(power);
		if (rememberPower) {
			previousArmPower = pwr;
		}
        climb.Spin(pwr);
    }

	private void RestoreArmMovement() {
		pivot.Spin(previousArmPower);
	}

    private void StopArmExtension(){
        //zero power value to stop arm extension
        climb.Spin(0);
    }

    private void RaiseCannon(double power, boolean rememberPower){
        //negative power value raises cannon
        double pwr;
        pwr = -Math.abs(power);
		if (rememberPower) {
			previousCannonPower = pwr;
		}
        pivot.Spin(pwr);  
    }

    private void LowerCannon(double power, boolean rememberPower){
        //positive power value lowers cannon
        double pwr;
        pwr = Math.abs(power);
		if (rememberPower) {
			previousCannonPower = pwr;
		}
        pivot.Spin(pwr);  
    }

	private void RestoreCannonMovement() {
		pivot.Spin(previousCannonPower);
	}

    private boolean IsCannonBelowDesiredAngle(double angle){
        boolean answer;
        double checkValue;
        checkValue = desiredAngle ;
        answer =  (angle < checkValue) ;
        return answer;
    }

    private boolean IsCannonAboveDesiredAngle(double angle){
        boolean answer;
        double checkValue;
        checkValue = desiredAngle ;
        answer =  (angle > checkValue) ;
        return answer;
    }

    private boolean IsAngleLessThanPivotMinSafe(double angle){
        boolean answer;
        double checkValue;
        checkValue = 50;
        answer = angle < checkValue;
        return answer;
    }

    private boolean IsAngleGreaterThanPivotMaxSafe(double angle){
        boolean answer;
        double checkValue;
        checkValue = 100;
        answer = angle > checkValue;
        return answer;
    }

    
    private boolean IsAngleLessThanPhysicalMinSafe(double angle){
        boolean answer;
        double checkValue;
        checkValue = 20;
        answer = angle < checkValue;
        return answer;
    }

    private boolean IsAngleGreaterThanPhysicalMaxSafe(double angle){
        boolean answer;
        double checkValue;
        checkValue = 120;
        answer = angle > checkValue;
        return answer;
    }

    private boolean IsAngleGreaterThanPhysicalMinSafe(double angle){
        boolean answer;
        answer = IsAngleLessThanPhysicalMinSafe(angle);
        return !answer;
    }

    private boolean IsAngleLessThanPhysicalMaxSafe(double angle){
        boolean answer;
        answer = IsAngleGreaterThanPhysicalMaxSafe(angle);
        return !answer;
    }

	private void SlowNearDesiredAngle(double angle) {
		double diff;
		double diffAV;
		diff = angle - desiredAngle;
		diffAV = Math.abs(diff);
		if (diff <= 0) { 
			// cannon is below desiredAngle
			// use stronger power values because cannon is working against gravity
			if (diffAV < 2){
				RaiseCannon(0.03, true);
			} else if (diffAV < 5) {
				RaiseCannon(0.04, true);
			} else if (diffAV < 10) {
				RaiseCannon(0.05, true);
			} else {
				RaiseCannon(pivotspeed, true);
			}
		} else { 
			// cannon is above desiredAngle
			// use smaller power levels because cannon will fall faster 
			// because cannon is working with gravity
			if (diffAV < 2){
				LowerCannon(0.01, true);
			} else if (diffAV < 5) {
				LowerCannon(0.02, true);
			} else if (diffAV < 10) {
				LowerCannon(0.03, true);
			} else {
				LowerCannon(pivotspeed, true);
			}
		}
	}
	


    private boolean climbArmSafetyUsed = false;
	private boolean climbCannonSafetyUsed = false;
	private double previousCannonPower = 0;
	private double previousArmPower = 0;
   

    public void Climb() {
        
        // Minimum climb power is 0.1
        // 1 inch is 8 revolutions
        
        double climbEncoder = Math.abs(climb.motor.inBuiltEncoder.getPosition());
        boolean climbArmSafeZone = false;
		boolean climbCannonSafeZone = false;
		


        if (climbing) {
            if (IsArmShorterThanLimit(climbEncoder)) {
                SmartDashboard.putString("executeArm", "Extend");
				climbArmSafetyUsed = true;
				climbArmSafeZone = false;
                ExtendArm(0.07, false);
            } else if (IsArmLongerThanLimit(climbEncoder)) {
                SmartDashboard.putString("executeArm", "Retract");
				climbArmSafetyUsed = true;
				climbArmSafeZone = false;
                RetractArm(0.07, false);
            } else {
                if (climbArmSafetyUsed) {
                    SmartDashboard.putString("executeArm", "Stop");
					climbArmSafetyUsed = false;
                    RestoreArmMovement();
                }
				climbArmSafeZone = true;
            }

            if (IsAngleLessThanPivotMinSafe(currentAngle)) {
				climbCannonSafetyUsed = true;
				climbCannonSafeZone = false;
                RaiseCannon(0.08, false);  
            } else if (IsAngleGreaterThanPivotMaxSafe(currentAngle)) {
				climbCannonSafetyUsed = true;
				climbCannonSafeZone = false;
                LowerCannon(0.08, false);  
            } else {
                if (climbCannonSafetyUsed) {
					climbCannonSafetyUsed = false;
					if (previousCannonPower == 0) {
						// 90 degrees the arm should be standing up vertically
						// try to hold cannon in place
						if (currentAngle < 90){
							RaiseCannon(0.01,false); 
						} else {
							LowerCannon(0.01, false); 
						}
					} else {
						RestoreCannonMovement();
					}
                }
				climbCannonSafeZone = true;
            }


            if (climbArmSafeZone && climbCannonSafeZone) {
                if ((currentState == robotState.idle || currentState == robotState.readyToShoot)) {
                    if (queuedState == robotState.climbingprep) {
                        if (IsCannonBelowDesiredAngle(currentAngle)) {
                           RaiseCannon(pivotspeed, true);
                        } 
                        if (IsCannonAboveDesiredAngle(currentAngle)) {
                           LowerCannon(pivotspeed * 0.5, true);
                        }
                        ExtendArm(climbSpeed, true);
                        currentState = robotState.climbingprep;
                    }
                }
                if (currentState == robotState.climbingprep) {
					SlowNearDesiredAngle(currentAngle);

                    if (IsArmExtened(climbEncoder)) {
 
						StopArmExtension();
                        climbDone = true;

                    }
                    if (IsCannonAboveDesiredAngle(currentAngle)) {

                        pivotDone = true;
                        RaiseCannon(0.01, true); //want to hold cannon in place
                        LockServo();


                    }
                    if (pivotDone && climbDone) {

                        currentState = robotState.readyToClimb;
                        climbing = false;
                        pivotDone = false;
                        climbDone = false;
                        RaiseCannon(0, true); //kill power to cannon it is no longer needed

                    }

                }
                
                if (currentState == robotState.readyToClimb) {

                    if (queuedState == robotState.climbing) {

                        RetractArm(climbSpeed, true);  
                        currentState = robotState.climbing;

                    }

                }
                if (currentState == robotState.climbing) {
                    if (IsArmRetracted(climbEncoder)) {

                        StopArmExtension();
                        climbDone = true;

                    }
                    if (climbDone) {

                        currentState = robotState.matchFinish;

                    }

                } 

            }

        }

    }

    private void Feed(double speed) {

        belt.Spin(speed);

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

    public void LockServo(){
        cannonLockRight.set(1);
        cannonLockLeft.set(1);
    }


    public void debugSmartDashboard() {

        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        SmartDashboard.putNumber("sensor", sensor.getVoltage());
        SmartDashboard.putString("currentstate", currentState.toString());
        SmartDashboard.putString("queuedstate", queuedState.toString());
        SmartDashboard.putNumber("Test", climb.motor.motor.getEncoder().getPosition());
        SmartDashboard.putNumber("Pid", pidCalcValue);
        SmartDashboard.putNumber("pidval", pidSetValue);
        SmartDashboard.putNumber("ClimbHall", Math.abs(climb.motor.inBuiltEncoder.getPosition()));
        SmartDashboard.putNumber("ClimbTarget", climbRevs);
        SmartDashboard.putBoolean("isPivoting", pivoting);

    }
    
}
