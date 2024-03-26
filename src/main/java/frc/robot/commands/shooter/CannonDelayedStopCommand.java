package frc.robot.commands.shooter;

import frc.robot.subsystems.ShooterSubsystem;

public class CannonDelayedStopCommand extends CannonStopCommand {
    private double delayRotations;
    private double currentCannonPosition;

    public CannonDelayedStopCommand(ShooterSubsystem shooterSubsystem, double delay) {
        super(shooterSubsystem);
        delayRotations = delay;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        currentCannonPosition = cannonRotation();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(currentCannonPosition - cannonRotation()) >= delayRotations;
    }

    @Override
    public void end(boolean interrupted) {
        m_ShooterSubsystem.stop();
    }
    
    private double cannonRotation() {
        return m_ShooterSubsystem.getCannonRotation();
    }
}
