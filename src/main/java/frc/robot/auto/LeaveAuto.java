package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class LeaveAuto extends SequentialCommandGroup {

    public LeaveAuto(DriveSubsystem drive) {

        addCommands(new AutoDrive(drive, 42.5, 270, 0.4, 0));

    }

}

