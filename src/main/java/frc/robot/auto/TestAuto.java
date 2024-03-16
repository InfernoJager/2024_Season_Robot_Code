package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotSubsystem;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(DriveSubsystem drive) {
        
        addCommands(new AutoDrive(drive));

    }

}
