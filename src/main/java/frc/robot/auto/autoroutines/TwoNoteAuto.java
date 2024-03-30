package frc.robot.auto.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.auto.autocommands.AutoCannonPreSpin;
import frc.robot.auto.autocommands.AutoDrive;
import frc.robot.auto.autocommands.AutoIntake;
import frc.robot.auto.autocommands.AutoShoot;
import frc.robot.subsystems.DriveSubsystem;

public class TwoNoteAuto extends SequentialCommandGroup{
    
    public TwoNoteAuto(RobotSubsystem robot, DriveSubsystem drive) {

        addCommands(
            new AutoShoot(robot, 65)
                .alongWith(new AutoCannonPreSpin(robot)),
            new AutoDrive(drive, 3, 270, 0.11, 0, 0),
            new AutoDrive(drive, 23, 270, 0.4, 0, 0)
                .alongWith(new AutoIntake(robot)),
            new AutoDrive(drive, 3, 90, 0.11, 0, 0),
            new AutoDrive(drive, 23, 90, 0.4, 0, 0)
                .alongWith(new AutoShoot(robot, 65), new AutoCannonPreSpin(robot))
        );

    }

}
