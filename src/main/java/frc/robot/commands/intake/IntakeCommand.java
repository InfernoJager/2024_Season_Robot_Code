package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    protected final IntakeSubsystem m_IntakeSubsystem;
    private final double m_intakeSpeed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
        m_IntakeSubsystem = intakeSubsystem;
        m_intakeSpeed = intakeSpeed;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.feed(m_intakeSpeed);
    }

    @Override
    public void end(boolean interrupted){
        m_IntakeSubsystem.stop();
    }
}
