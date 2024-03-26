package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motor.PairedMotors;

public class ShooterSubsystem extends SubsystemBase {
    public final PairedMotors cannon;

    public ShooterSubsystem() {
        cannon = new PairedMotors(Constants.CANNON_MAIN, Constants.CANNON_SLAVE, false, false);
        cannon.SetRampRate(0.75);
    }

    public void start(double speed) {
        cannon.Spin(-Math.abs(speed));
    }

    public void stop() {
        cannon.Spin(0);
    }

    public double getCannonRotation() {
        return cannon.mainMotor.inBuiltEncoder.getPosition();
    }

    public boolean isCannonPrepped(double targetSpeed) {
        return cannon.mainMotor.inBuiltEncoder.getVelocity() <= -Math.abs(targetSpeed);
    }

    public BooleanSupplier isCannonPreppedSupplier(double targetSpeed) {
        return () -> isCannonPrepped(targetSpeed);
    }
}
