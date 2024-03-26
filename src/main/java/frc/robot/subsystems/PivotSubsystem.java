package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motor.PairedMotors;

public class PivotSubsystem extends SubsystemBase {
    public final PairedMotors pivot;
    private double pivotSpeed;
    private PIDController pivotpid;
    private boolean pivotActive;
    private double desiredAngle;

    private double angleOffset = 20;

    public PivotSubsystem() {
        pivotSpeed = 0;

        pivot = new PairedMotors(Constants.PIVOT_MAIN, Constants.PIVOT_SLAVE, false, true);
        pivot.mainMotor.absoluteEncoder.setInverted(true);
        pivot.SetRampRate(0.1);

        pivotpid = new PIDController(0.01, 0, 0.0001);
        pivotpid.enableContinuousInput(0, 360);
        pivotpid.setTolerance(0.25);

        pivotActive = false;
    }

    public void stop() {
        spin(0);
        pivotActive = false;
    }

    public void setAngle(double angle) {
        pivotActive = true;
        desiredAngle = angle;
    }

    public void setPivotSpeed(double speed) {
        pivotSpeed = speed;
    }

    public double currentAngle() {
        return (pivot.mainMotor.getAbsoluteRawAngle() + angleOffset) % 360;
    }

    public boolean isNearDesiredAngle(double deadzone) {
        double angle = currentAngle();
        return isInDeadzone(angle, deadzone);
    }

    public void pivot() {
        if (!pivotActive) return;
        
        double angle = currentAngle();
        double minAngle = 20;
        double maxAngle = 120;

        if (angle < minAngle) {
            pivot.Spin(-0.03);
        }
        if (angle > maxAngle) {
            pivot.Spin(0.03);
        }

        double pidValue = getPIDValue(angle);
        double speedMultiplier = speedMultiplier(pidValue);

        spin(pidValue * speedMultiplier);
    }

    private void spin(double speed) {
        pivot.Spin(speed);
    }

    private double speedMultiplier(double pidValue) {
        double angle = currentAngle();

        if (isInDeadzone(angle, 0.25)) {
            return 1;
        }
        
        if (Math.abs(pidValue) < 0.002) {
            return 13;
        } else if (Math.abs(pidValue) < 0.003) {
            return 9;
        } else if (Math.abs(pidValue) < 0.005) {
            return 6;
        } else if (Math.abs(pidValue) < 0.01) {
            return 3;
        } else if (Math.abs(pidValue) < 0.03) {
            return 2;
        }

        return 1;
    }

    private boolean isInDeadzone(double angle, double deadzone) {
        return desiredAngle > angle - deadzone && desiredAngle < angle + deadzone;
    }

    private double getPIDValue(double angle) {
        double pidBaseValue = pivotpid.calculate(angle, desiredAngle);
        double pidSetValue;
        if (pidBaseValue > 0) {
            pidSetValue = Math.min(pidBaseValue, -pivotSpeed);  
        } else {
            pidSetValue = Math.max(pidBaseValue, pivotSpeed);
        }
        if (desiredAngle >= angle) {
            return -Math.abs(pidSetValue);
        } else {
            return Math.abs(pidSetValue);
        }
    }
}
