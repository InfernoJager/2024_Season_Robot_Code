package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class SetPivotAngleCommand extends Command {
    protected final PivotSubsystem m_PivotSubsystem;
    private final double m_angle;
    private final double pivotSpeed;

    public SetPivotAngleCommand(PivotSubsystem pivotSubsystem, double angle, double speed) {
        m_PivotSubsystem = pivotSubsystem;
        m_angle = angle;
        pivotSpeed = speed;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        m_PivotSubsystem.setPivotSpeed(pivotSpeed);
        m_PivotSubsystem.setAngle(m_angle);
    }

    @Override
    public boolean isFinished() {
       return true;
    }
}
