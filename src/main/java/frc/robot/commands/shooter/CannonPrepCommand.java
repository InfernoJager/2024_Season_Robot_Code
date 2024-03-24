package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class CannonPrepCommand extends Command {
    protected final ShooterSubsystem m_ShooterSubsystem;
    private final double shootingSpeed;

    public CannonPrepCommand(ShooterSubsystem shooterSubsystem, double speed) {
        m_ShooterSubsystem = shooterSubsystem;
        shootingSpeed = speed;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.start(shootingSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
