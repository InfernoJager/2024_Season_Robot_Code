package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class CannonLoadCommand extends Command {
    protected final FeederSubsystem m_FeederSubsystem;
    private final double feedSpeed;

    public CannonLoadCommand(FeederSubsystem feederSubsystem, double speed) {
        m_FeederSubsystem = feederSubsystem;
        feedSpeed = speed;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        m_FeederSubsystem.load(feedSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
