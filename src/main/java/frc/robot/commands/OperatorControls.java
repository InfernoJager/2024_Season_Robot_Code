package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import javax.swing.text.StyleContext.SmallAttributeSet;

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

        //double manualClimb = Math.abs(robot.climb.motor.inBuiltEncoder.getPosition());

        //Presets
        // Button board layout from top down
        // 4 3 2 1
        // 8 7 6 5
        final int intake = 1;
        final int climbUp = 3; // R
        final int climbing = 2; // X
        final int speakerShoot = 4; // A
        final int cancelOperation = 6; // Start
        final int trapShoot = 7;
        final int ampShoot = 8; // B

        /*Operator Xbox Controller*/
        // if (controller.getLeftY() < -0.1 && manualClimb < 108) {
        //     robot.climb.Spin(-0.1);
        // } else if (controller.getLeftY() > 0.1 && manualClimb > 4) {
        //     robot.climb.Spin(0.1);
        // }

        /*Operator Button Board*/
        if (buttonBoard.getRawButton(intake)) {
            SmartDashboard.putString("buttonPressed", "Intake");
            robot.SetIntakeSpeed(0.4);
            robot.SetPivotSpeed(-1);
            robot.SetDesiredAngle(21);
            robot.SetQueuedState(robotState.intakingPivot);
        }
        if (buttonBoard.getRawButtonPressed(climbing)) {
            SmartDashboard.putString("buttonPressed", "Climbing");
            robot.SetQueuedState(robotState.climbing);
            robot.SetClimbSpeed(1);
            robot.SetDesriedClimb(3);
            robot.ClimbStart();
        }
        if (buttonBoard.getRawButtonPressed(climbUp)) {
            SmartDashboard.putString("buttonPressed", "Climbing Prep");
            robot.SetQueuedState(robotState.climbingprep);
            robot.SetClimbSpeed(1);
            robot.SetDesiredAngle(66);
            robot.SetPivotSpeed(0.11);
            robot.SetDesriedClimb(13);
            robot.ClimbStart();
        }
        if (buttonBoard.getRawButtonPressed(speakerShoot)) {
            SmartDashboard.putString("buttonPressed", "Speaker Shoot");
            robot.SetQueuedState(robotState.speakerShooting);
            robot.SetDesiredAngle(65);
            robot.SetTargetAngle(65);
            robot.SetPivotSpeed(-1);
            robot.SetShootSpeed(-1);
        }
        if (buttonBoard.getRawButtonPressed(cancelOperation)) {
            SmartDashboard.putString("buttonPressed", "Cancel");
            robot.resetMotors();
            robot.revertStates();
        }
        // if (buttonBoard.getRawButtonPressed(trapShoot)) {
        //     SmartDashboard.putString("buttonPressed", "Trap Shoot");
        //     robot.SetQueuedState(robotState.speakerShooting);
        //     robot.SetDesiredAngle(97);
        //     robot.SetTargetAngle(97);
        //     robot.SetPivotSpeed(-1);
        //     robot.SetShootSpeed(-1);
        // }
        if (buttonBoard.getRawButtonPressed(ampShoot)) {
            SmartDashboard.putString("buttonPressed", "Amp Shoot");
            robot.SetQueuedState(robotState.ampShooting);
            robot.SetDesiredAngle(80);
            robot.SetPivotSpeed(-0.5);
            robot.SetShootSpeed(-0.13);
        }
        
        robot.NoteBack();
        robot.Intake();
        robot.Climb();
        robot.Shoot();
        robot.Pivot();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
