package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.IntakeLoadingCommand;
import frc.robot.commands.feeder.NotePrepCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.pivot.ReachPivotAngleCommand;
import frc.robot.commands.pivot.SetPivotAngleCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PickupCommand extends SequentialCommandGroup {
    public PickupCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem) {
        double intakeAngle = 21;
        double intakeSpeed = 0.5;
        double idleAngle = 33;

        addCommands(
            new ReachPivotAngleCommand(pivotSubsystem, intakeAngle, 1, 2.5),
            new IntakeCommand(intakeSubsystem, intakeSpeed).raceWith(
                new IntakeLoadingCommand(feederSubsystem, intakeSpeed)
            ),
            new NotePrepCommand(feederSubsystem).alongWith(
                new SetPivotAngleCommand(pivotSubsystem, idleAngle, 1)
            )
        );

        addRequirements(intakeSubsystem, feederSubsystem, pivotSubsystem);
    }
}