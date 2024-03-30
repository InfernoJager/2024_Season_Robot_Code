package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotSubsystem;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(DriveSubsystem drive, RobotSubsystem robot) {

        addCommands(new AutoDrive(drive, 3, 227, 0.11, 0, 0));
        addCommands(new AutoDrive(drive, 39, 227, 0.35, -0.125, 160).alongWith(new AutoIntake(robot)));
        addCommands(new AutoDrive(drive, 7.5, 32, 0.4, 0, 0));
        addCommands(new AutoDrive(drive, 34.5, 32, 0.4, 0.125, 180).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));
        addCommands(new AutoDrive(drive, 3, 310, 0.11, 0, 0));
        addCommands(new AutoDrive(drive, 39, 310, 0.35, 0.125, 200).alongWith(new AutoIntake(robot)));
        addCommands(new AutoDrive(drive, 7.5, 145, 0.4, 0, 0));
        addCommands(new AutoDrive(drive, 34.5, 145, 0.4, -0.125, 180).alongWith(new AutoShoot(robot, 65)).alongWith(new AutoCannonPreSpin(robot)));

    }

}
