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

        //double manualClimb = Math.abs(robot.climb.motor.inBuiltEncoder.getPosition());

        //Presets
        // Button board layout from top down
        // 4 3 2 1
        // 8 7 6 5
        final int climbUp = 3; // R
        final int climbing = 2; // X
        final int speakerShoot = 4; // A
        final int cancelOperation = 6; // Start
        final int ampShoot = 8; // B
        final int test = 1;//Y

        /*Operator Xbox Controller*/
        // if (controller.getLeftY() < -0.1 && manualClimb < 108) {
        //     robot.climb.Spin(-0.1);
        // } else if (controller.getLeftY() > 0.1 && manualClimb > 4) {
        //     robot.climb.Spin(0.1);
        // }

        /*Operator Button Board*/
        if (buttonBoard.getRawButtonPressed(climbUp)) {
            SmartDashboard.putString("buttonPressed", "climbingprep");
            robot.SetQueuedState(robotState.climbingprep);
            robot.SetClimbSpeed(0.5);
            robot.SetDesiredAngle(66);
            robot.SetPivotSpeed(0.11);
            robot.SetDesriedClimb(13.25);
            robot.ClimbStart();
        }
        if (buttonBoard.getRawButtonPressed(climbing)) {
            SmartDashboard.putString("buttonPressed", "climb");
            robot.SetQueuedState(robotState.climbing);
            robot.SetClimbSpeed(0.75);
            // robot.SetPivotSpeed(-0.05);
            // robot.SetDesiredAngle(90);
            robot.SetDesriedClimb(3 );
            robot.ClimbStart();
        }
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
            robot.SetDesiredAngle(117.5);
            robot.SetPivotSpeed(-0.3);
            robot.SetShootSpeed(-0.12);
        }
        //if (buttonBoard.getRawButtonPressed(test)) {
        //    SmartDashboard.putString("buttonPressed", "test Servo");
        //    robot.LockServo();
        //}

        robot.Climb();
        robot.Shoot();
        robot.Pivot();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
