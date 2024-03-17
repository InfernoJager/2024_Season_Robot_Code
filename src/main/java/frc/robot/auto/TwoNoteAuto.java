package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TwoNoteAuto extends SequentialCommandGroup{
    
    public TwoNoteAuto(RobotSubsystem robot, DriveSubsystem drive) {

        addCommands(new AutoShoot(robot, 65).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 1, 270, 0.11));
        addCommands(new AutoDrive(drive, 25, 270, 0.30).alongWith(new AutoIntake(robot)));
        addCommands(new AutoShoot(robot, 53.8).alongWith(new AutoCannonPreSpin(robot)));

    }

}
