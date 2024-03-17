package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.RobotSubsystem.robotState;

public class AutoCannonPreSpin extends Command {
    
    private final RobotSubsystem m_robot;
    
    public AutoCannonPreSpin(RobotSubsystem robot) {

        m_robot = robot;

    }

    @Override
    public void initialize() {
        
        m_robot.SetShootSpeed(-1);

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
