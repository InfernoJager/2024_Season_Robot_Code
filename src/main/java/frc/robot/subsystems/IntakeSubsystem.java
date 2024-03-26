package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motor.Motors;

public class IntakeSubsystem extends SubsystemBase {
    private final Motors intake;

    public IntakeSubsystem() {
        intake = new Motors(Constants.INTAKE, false, false);
    }

    public void feed(double speed) {
        intake.Spin(Math.abs(speed));
    }

    public void purge(double speed) {
        intake.Spin(-Math.abs(speed));
    }

    public void stop() {
        intake.Spin(0);
    }
}
