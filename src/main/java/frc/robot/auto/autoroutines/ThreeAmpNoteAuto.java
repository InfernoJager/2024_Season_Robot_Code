package frc.robot.auto.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.auto.autocommands.AutoCannonPreSpin;
import frc.robot.auto.autocommands.AutoDrive;
import frc.robot.auto.autocommands.AutoIntake;
import frc.robot.auto.autocommands.AutoShoot;
import frc.robot.subsystems.DriveSubsystem;

public class ThreeAmpNoteAuto  extends SequentialCommandGroup{
    
    public ThreeAmpNoteAuto(DriveSubsystem drive, RobotSubsystem robot) {

        addCommands(
            new AutoShoot(robot, 65)
                .alongWith(new AutoCannonPreSpin(robot)),
            new AutoDrive(drive, 3, 270, 0.11, 0, 0),
            new AutoDrive(drive, 23, 270, 0.4, 0, 0)
                .alongWith(new AutoIntake(robot)),
            new AutoDrive(drive, 3, 90, 0.11, 0, 0),
            new AutoDrive(drive, 23, 90, 0.4, 0, 0)
                .alongWith(new AutoShoot(robot, 65), new AutoCannonPreSpin(robot)),
            new AutoDrive(drive, 3, 227, 0.11, 0, 0),
            new AutoDrive(drive, 39, 227, 0.35, -0.125, 160)
                .alongWith(new AutoIntake(robot)),
            new AutoDrive(drive, 7.5, 32, 0.4, 0, 0),
            new AutoDrive(drive, 34.5, 32, 0.4, 0.125, 180)
                .alongWith(new AutoShoot(robot, 65), new AutoCannonPreSpin(robot))
        );
    }

}
