package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotSubsystem;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(DriveSubsystem drive, RobotSubsystem robot) {

        addCommands(new AutoShoot(robot, 65));

    }

}
