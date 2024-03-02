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
        final int climbUp = 2;
        final int climbHigh = 3;
        final int speakerShoot = 4;
        final int cancelOperation = 6;
        final int climbLow = 7;
        final int ampShoot = 8;

        /*Operator Xbox Controller*/

        /*Operator Button Board*/
        if (buttonBoard.getRawButtonPressed(speakerShoot)) {
            SmartDashboard.putString("buttonPressed", "speakerShoot");
            robot.SetQueuedState(robotState.speakerShooting);
            robot.SetDesiredAngle(61);
            robot.SetPivotSpeed(-0.3);
            robot.SetShootSpeed(-1);
        }
        if (buttonBoard.getRawButtonPressed(cancelOperation)) {
            SmartDashboard.putString("buttonPressed", "safeAngle");
            robot.resetMotors();
            robot.revertStates();
            robot.SetDesiredAngle(33);
            robot.SetPivotSpeed(-0.3);
            robot.PivotStart();
        }
        if (buttonBoard.getRawButtonPressed(ampShoot)) {
            SmartDashboard.putString("buttonPressed", "ampShoot");
            robot.SetQueuedState(robotState.ampShooting);
            robot.SetDesiredAngle(117);
            robot.SetPivotSpeed(-0.3);
            robot.SetShootSpeed(-0.12);
        }

        robot.Shoot();
        robot.Pivot();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
