package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motor.Motors;

public class FeederSubsystem extends SubsystemBase {
    private final Motors belt;
    private final AnalogInput sensor;
    private boolean hasUnprepedNote;

    public FeederSubsystem() {
        belt = new Motors(Constants.FEEDER_BELT, false, false);
        sensor = new AnalogInput(0);
        hasUnprepedNote = false;
    }

    public void load(double speed) {
        belt.Spin(-Math.abs(speed));
    }

    public void retract(double speed) {
        belt.Spin(Math.abs(speed));
    }

    public void stop() {
        belt.Spin(0);
    }

    public boolean isNoteIn() {
        return sensor.getVoltage() < 0.01;
    }

    public BooleanSupplier isNoteInSupplier() {
        return () -> isNoteIn();
    }

    public boolean isNoteOut() {
        return sensor.getVoltage() > 0.1;
    }

    public BooleanSupplier isNoteOutSupplier() {
        return () -> isNoteOut();
    }

    public boolean hasUnpreppedNote() {
        return hasUnprepedNote;
    }

    public void setUnpreppedNote() {
        hasUnprepedNote = true;
    }

    public void setPreppedNote() {
        hasUnprepedNote = false;
    }
}
