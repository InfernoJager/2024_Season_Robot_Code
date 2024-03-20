package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SideAuto extends SequentialCommandGroup{
    
    public SideAuto(RobotSubsystem robot, DriveSubsystem drive) {

        addCommands(new AutoShoot(robot, 65).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 1, 270, 0.11, 0));
        addCommands(new AutoDrive(drive, 25, 270, 0.40, 0).alongWith(new AutoIntake(robot)));
        addCommands(new AutoDrive(drive, 26, 90, 0.40, 0).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));

    }

}