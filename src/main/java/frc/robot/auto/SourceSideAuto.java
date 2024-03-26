package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SourceSideAuto extends SequentialCommandGroup{
    
    public SourceSideAuto(RobotSubsystem robot, DriveSubsystem drive) {

        if (DriverStation.getAlliance().isPresent()){
        
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                addCommands(new AutoShoot(robot, 65).alongWith(new AutoCannonPreSpin(robot)));
                addCommands(new AutoDrive(drive, 30, 330, 0.4, 0));
                addCommands(new AutoDrive(drive, 7.5, 0, 0, -0.125));
            } else {
                addCommands(new AutoShoot(robot, 65).alongWith(new AutoCannonPreSpin(robot)));
                addCommands(new AutoDrive(drive, 30, 210, 0.4, 0));
                addCommands(new AutoDrive(drive, 7.5, 0, 0, 0.125));
            }

        }

    }

}