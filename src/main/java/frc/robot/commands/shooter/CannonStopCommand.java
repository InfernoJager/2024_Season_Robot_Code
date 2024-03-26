package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class CannonStopCommand extends Command {
    protected final ShooterSubsystem m_ShooterSubsystem;

    public CannonStopCommand(ShooterSubsystem shooterSubsystem) {
        m_ShooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
