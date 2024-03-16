package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSubsystem;

public class TwoNoteAuto extends SequentialCommandGroup{
    
    public TwoNoteAuto(RobotSubsystem robot) {

        addCommands(new AutoShoot(robot));

    }

}
