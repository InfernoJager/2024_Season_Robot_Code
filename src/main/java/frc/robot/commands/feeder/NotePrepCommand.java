package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class NotePrepCommand extends Command {
    protected final FeederSubsystem m_FeederSubsystem;

    public NotePrepCommand(FeederSubsystem feederSubsystem) {
        m_FeederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        m_FeederSubsystem.retract(0.25);
    }

    @Override
    public boolean isFinished() {
        return m_FeederSubsystem.isNoteOut();
    }

    @Override
    public void end(boolean interrupted){
        m_FeederSubsystem.stop();

        if (!interrupted) {
            m_FeederSubsystem.setPreppedNote();
        }
    }
}
