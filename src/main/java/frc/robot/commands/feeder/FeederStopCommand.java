package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class FeederStopCommand extends Command {
    protected final FeederSubsystem m_FeederSubsystem;

    public FeederStopCommand(FeederSubsystem feederSubsystem) {
        m_FeederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        m_FeederSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
