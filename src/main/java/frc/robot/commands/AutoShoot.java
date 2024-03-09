package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.RobotSubsystem.robotState;

public class AutoShoot extends Command{
    private RobotSubsystem robot;

    public AutoShoot(RobotSubsystem robot){
        this.robot = robot;
    }

    @Override
    public void initialize(){
        robot.resetStates();
        robot.SetRobotCurrentState(robotState.readyToShoot);
        robot.SetDesiredAngle(61);
        robot.SetPivotSpeed(-0.3);
        robot.SetShootSpeed(-1);
        robot.SetQueuedState(robotState.speakerShooting);
    }

    @Override
    public void execute(){
        robot.Shoot();
    }

    @Override
    public boolean isFinished() {
        return (robot.GetRobotCurrentState() == robotState.idle);
    }

    @Override
    public void end(boolean interrupted){
        robot.resetStates();
    }

}