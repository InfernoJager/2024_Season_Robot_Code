package frc.robot.auto.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.autocommands.AutoCannonPreSpin;
import frc.robot.auto.autocommands.AutoShoot;
import frc.robot.subsystems.RobotSubsystem;

public class AmpSideAuto extends SequentialCommandGroup{
    
    public AmpSideAuto(RobotSubsystem robot) {

        addCommands(
            new AutoShoot(robot, 65)
                .alongWith(new AutoCannonPreSpin(robot))
        );
    
    }

}