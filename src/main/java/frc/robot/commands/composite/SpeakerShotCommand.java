package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.feeder.CannonLoadCommand;
import frc.robot.commands.feeder.FeederStopCommand;
import frc.robot.commands.pivot.ReachPivotAngleCommand;
import frc.robot.commands.pivot.SetPivotAngleCommand;
import frc.robot.commands.shooter.CannonPrepCommand;
import frc.robot.commands.shooter.CannonStopCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SpeakerShotCommand extends SequentialCommandGroup {

    public SpeakerShotCommand(FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
        double shotAngle = 65;
        double pivotSpeed = 0.5;
        double shotSpeed = 1;
        double cannonSpeed = 400;
        double idleAngle = 33;

        addCommands(
            new ReachPivotAngleCommand(pivotSubsystem, shotAngle, pivotSpeed, 0.7),
            new CannonPrepCommand(shooterSubsystem, shotSpeed).until(shooterSubsystem.isCannonPreppedSupplier(cannonSpeed)),
            new CannonLoadCommand(feederSubsystem, 1),
            new WaitUntilCommand(feederSubsystem.isNoteInSupplier()),
            new WaitUntilCommand(feederSubsystem.isNoteOutSupplier()),
            new CannonStopCommand(shooterSubsystem).alongWith(
                new FeederStopCommand(feederSubsystem),
                new SetPivotAngleCommand(pivotSubsystem, idleAngle, 1)
            )
        );

        addRequirements(feederSubsystem, pivotSubsystem, shooterSubsystem);
    }
}