package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class IntakeLoadingCommand extends Command {
    protected final FeederSubsystem m_FeederSubsystem;
    private final double m_intakeSpeed;

    public IntakeLoadingCommand(FeederSubsystem feederSubsystem, double intakeSpeed) {
        m_FeederSubsystem = feederSubsystem;
        m_intakeSpeed = intakeSpeed;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        m_FeederSubsystem.load(m_intakeSpeed * 0.6);
    }

    @Override
    public boolean isFinished() {
        return m_FeederSubsystem.isNoteIn();
    }

    @Override
    public void end(boolean interrupted){
        m_FeederSubsystem.stop();

        if (!interrupted) {
            m_FeederSubsystem.setUnpreppedNote();
        }
    }
}
