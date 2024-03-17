package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.RobotSubsystem.robotState;

public class AutoIntake extends Command {
    
    private final RobotSubsystem m_robot;

    public AutoIntake(RobotSubsystem robot) {

        m_robot = robot;

    }

    @Override
    public void initialize() {

        m_robot.SetIntakeSpeed(0.5);
        m_robot.SetPivotSpeed(-0.3);
        m_robot.SetDesiredAngle(21);
        m_robot.SetQueuedState(robotState.intakingPivot);

    }

    @Override
    public void execute() {
        m_robot.NoteBack();
    }

    @Override
    public boolean isFinished() {
        return (m_robot.GetRobotCurrentState() == robotState.readyToShoot);
    }

    @Override
    public void end(boolean interrupted){
        m_robot.resetStates();
    }

}
