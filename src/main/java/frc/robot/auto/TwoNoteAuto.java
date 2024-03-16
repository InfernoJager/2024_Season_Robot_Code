package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class TwoNoteAuto extends SequentialCommandGroup{
    
    public TwoNoteAuto(DriveSubsystem drive) {

        addCommands(new AutoDrive(drive));

    }

}
