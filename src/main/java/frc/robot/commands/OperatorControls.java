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
        boolean isShootReady = false;
        final int ampPreset = 1;
        final int speakerShoot = 5;
        final int ampShoot = 6;

        /*Operator Xbox Controller*/

        /*Operator Button Board*/
        if (buttonBoard.getRawButtonPressed(ampPreset)) {
            SmartDashboard.putString("key", "ampPreset");
            robot.PivotStart();
            robot.SetDesiredAngle(119);
        }
        if (buttonBoard.getRawButtonPressed(speakerShoot)) {
            SmartDashboard.putString("key", "speakerShoot");
            robot.Feed(0.05);
            robot.PivotStart();
            robot.SetDesiredAngle(0);
        }
        if (buttonBoard.getRawButtonPressed(ampShoot)) {
            SmartDashboard.putString("key", "ampShoot");
            robot.Feed(0.05);
            robot.Shoot(0.15, robotState.ampShooting);
        }

        robot.ShootStop(3);
        robot.ClimbStop(3);
        robot.Pivot(robot.pivot.mainMotor.getAbsoluteRawAngle(), 0.05);

        if (robot.readyToShoot) {
            robot.Shoot(0.5, robotState.speakerShooting);
        }
            
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
