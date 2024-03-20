package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.RobotSubsystem.robotState;

public class AutoShoot extends Command{
    private final RobotSubsystem m_robot;
    private double shootAngle;

    public AutoShoot(RobotSubsystem robot, double angle) {

        m_robot = robot;
        shootAngle = angle;

    }

    @Override
    public void initialize() {
        
        m_robot.SetRobotCurrentState(robotState.readyToShoot);
        m_robot.SetDesiredAngle(shootAngle);
        m_robot.SetTargetAngle(shootAngle);
        m_robot.SetPivotSpeed(-1);
        m_robot.SetShootSpeed(-1);
        m_robot.SetQueuedState(robotState.speakerShooting);

    }

    @Override
    public void execute() {
        m_robot.Shoot();
    }

    @Override
    public boolean isFinished() {
        return (m_robot.GetRobotCurrentState() == robotState.idle);
    }

    @Override
    public void end(boolean interrupted){
        m_robot.resetStates();
    }

}
