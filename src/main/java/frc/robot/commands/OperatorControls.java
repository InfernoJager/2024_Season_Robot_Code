package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.subsystems.RobotSubsystem.robotState;

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
        final int speakerShoot = 4;
        final int safeAngle = 6;
        final int ampShoot = 8;

        /*Operator Xbox Controller*/

        /*Operator Button Board*/
        if (buttonBoard.getRawButtonPressed(speakerShoot)) {
            SmartDashboard.putString("buttonPressed", "speakerShoot");
            robot.SetQueuedState(robotState.speakerShootingPrep);
            robot.SetDesiredAngle(30);
            robot.SetPivotSpeed(0.05);
            robot.SetShootSpeed(0.05);
        }
        if (buttonBoard.getRawButtonPressed(safeAngle)) {
            SmartDashboard.putString("buttonPressed", "safeAngle");
            robot.SetDesiredAngle(20);
            robot.SetPivotSpeed(0.05);
        }
        if (buttonBoard.getRawButtonPressed(ampShoot)) {
            SmartDashboard.putString("buttonPressed", "ampShoot");
            robot.SetQueuedState(robotState.ampShooting);
            robot.SetDesiredAngle(119);
            robot.SetPivotSpeed(0.05);
            robot.SetShootSpeed(0.05);
        }

        robot.ClimbStop(3);
        robot.Shoot();
        robot.Pivot();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
