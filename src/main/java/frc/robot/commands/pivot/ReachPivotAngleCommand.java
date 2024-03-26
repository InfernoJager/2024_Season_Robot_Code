package frc.robot.commands.pivot;

import frc.robot.subsystems.PivotSubsystem;

public class ReachPivotAngleCommand extends SetPivotAngleCommand {
    private final double m_deadzone;

    public ReachPivotAngleCommand(PivotSubsystem pivotSubsystem, double angle, double pivotSpeed, double deadzone) {
        super(pivotSubsystem, angle, pivotSpeed);

        m_deadzone = deadzone;
    }

    @Override
    public boolean isFinished() {
       return m_PivotSubsystem.isNearDesiredAngle(m_deadzone);
    }
}
