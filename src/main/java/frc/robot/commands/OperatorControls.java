package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.RobotSubsystem;

public class OperatorControls extends Command{
    
    private final XboxController controller;
    private final GenericHID buttonBoard;
    private final RobotSubsystem robot;

    public OperatorControls(RobotSubsystem robot, XboxController operatorController, GenericHID buttonBoard) {

        this.robot = robot;
        this.controller = operatorController;
        this.buttonBoard = buttonBoard;

        addRequirements(robot);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        //Presets
        final int ampPreset = 1;
        final int intakePreset = 3;
        final int intake = 4;
        final int speakerShoot = 5;
        final int ampShoot = 6;
        final int stuckNotePreset = 7;
        final int stuckNoteOuttake = 8;

        /*Operator Xbox Controller*/

        /*Operator Button Board*/
        if (buttonBoard.getRawButtonPressed(ampPreset)) {
            robot.Pivot(0, 119, 0.05);
        }
        if (buttonBoard.getRawButtonPressed(intakePreset)) {
            robot.Pivot(0, 20, 0.05);
        }
        if (buttonBoard.getRawButton(intake)) {
            robot.Intake(0.05);
            robot.Feed(0.05, false);
        }
        if (buttonBoard.getRawButton(speakerShoot)) {
            robot.PivotStart();
        }
        if (buttonBoard.getRawButton(ampShoot)) {
            robot.Shoot(0.15);
        }
        if (buttonBoard.getRawButton(stuckNotePreset)) {
            robot.Pivot(0, 90, 0.05);
        }
        if (buttonBoard.getRawButton(stuckNoteOuttake)) {
            robot.Intake(-0.05);
        }
        
        robot.ShootStop(3);
        robot.IntakeStop(3);
        robot.ClimbStop(3);
        robot.Pivot(robot.pivot.mainMotor.getAbsoluteRawAngle(), 0, 0.05);
        
        if (robot.readyToShoot) {
            robot.Shoot(0.5);
        }
            
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
