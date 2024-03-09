package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.swervemodule.SwerveModule;
import frc.robot.swervemodule.SwerveModules;

public class AutoSequentialGroup extends SequentialCommandGroup {


    public AutoSequentialGroup(DriveSubsystem drive, RobotSubsystem rbt) {
        //addCommands(new AutoDrive(drive));
        addCommands(new AutoShoot(rbt));
    }


 

 }
