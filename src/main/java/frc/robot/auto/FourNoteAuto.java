package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class FourNoteAuto  extends SequentialCommandGroup{
    
    public FourNoteAuto(DriveSubsystem drive, RobotSubsystem robot) {

        addCommands(new AutoShoot(robot, 65).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 3, 270, 0.11, 0));
        addCommands(new AutoDrive(drive, 23, 270, 0.4, 0).alongWith(new AutoIntake(robot)));
        addCommands(new AutoDrive(drive, 3, 90, 0.11, 0));
        addCommands(new AutoDrive(drive, 23, 90, 0.4, 0).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 3, 212.9, 0.11, 0));
        addCommands(new AutoDrive(drive, 39, 212.9, 0.35, 0).alongWith(new AutoIntake(robot)));
        addCommands(new AutoDrive(drive, 7.5, 32.9, 0.4, 0));
        addCommands(new AutoDrive(drive, 31.5, 32.9, 0.4, 0).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 3, 320.25, 0.11, 0));
        addCommands(new AutoDrive(drive, 39, 320.25, 0.35, 0).alongWith(new AutoIntake(robot)));
        addCommands(new AutoDrive(drive, 7.5, 145.25, 0.4, 0));
        addCommands(new AutoDrive(drive, 32.5, 145.25, 0.4, 0).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));

    }

}