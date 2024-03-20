package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class FourNoteAuto  extends SequentialCommandGroup{
    
    public FourNoteAuto(DriveSubsystem drive, RobotSubsystem robot) {

        addCommands(new AutoShoot(robot, 65).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 3, 270, 0.11, 0));
        addCommands(new AutoDrive(drive, 23, 270, 0.4, 0).alongWith(new AutoIntake(robot)));
        // Angle 89.75 if necessary as an offset
        addCommands(new AutoDrive(drive, 3, 90, 0.11, 0));
        addCommands(new AutoDrive(drive, 23, 90, 0.4, 0).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 3, 213.5, 0.11, 0));
        addCommands(new AutoDrive(drive, 36, 213.5, 0.35, 0).alongWith(new AutoIntake(robot)));
        addCommands(new AutoDrive(drive, 7.5, 33.5, 0.4, 0));
        addCommands(new AutoDrive(drive, 31.5, 33.5, 0.4, 0).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 3, 319.25, 0.11, 0));
        addCommands(new AutoDrive(drive, 36, 319.25, 0.35, 0).alongWith(new AutoIntake(robot)));
        addCommands(new AutoDrive(drive, 7.5, 145, 0.4, 0));
        addCommands(new AutoDrive(drive, 31.5, 145, 0.4, 0).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));

    }

}