package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.RobotSubsystem;
import edu.wpi.first.wpilibj.AnalogInput;

public class OperatorControls extends Command{
    
    private final XboxController controller;
    private final GenericHID buttonBoard;
    private final RobotSubsystem robot;
    private final AnalogInput sensor;

    public OperatorControls(RobotSubsystem robot, XboxController operatorController, GenericHID buttonBoard) {

        this.robot = robot;
        this.controller = operatorController;
        this.buttonBoard = buttonBoard;
        this.sensor = new AnalogInput(0);

        addRequirements(robot);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        //Presets
        boolean note = false;
        boolean isShootReady = false;
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
            SmartDashboard.putString("key", "ampPreset");
            robot.PivotStart();
            robot.SetDesiredAngle(119);
        }
        if (buttonBoard.getRawButtonPressed(intakePreset)) {
            SmartDashboard.putString("key", "intakePreset");
            robot.PivotStart();
            robot.SetDesiredAngle(20);
        }
        if (buttonBoard.getRawButtonPressed(intake)) {
            SmartDashboard.putString("key", "intake");
            robot.Intake(0.05);
            robot.Feed(0.05);
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
            robot.Shoot(0.15);
        }
        if (buttonBoard.getRawButtonPressed(stuckNotePreset)) {
            SmartDashboard.putString("key", "stuckNotePreset");
            robot.PivotStart();
            robot.SetDesiredAngle(90);
        }
        if (buttonBoard.getRawButtonPressed(stuckNoteOuttake)) {
            SmartDashboard.putString("key", "stuckNoteOuttake");
            robot.Intake(-0.05);
        }
        
        if (robot.IsIntakeFinished(3)) {
            note = !isNoteReady(sensor);
        }
        if (robot.IsShootFinished(3)) {
            note = isNoteReady(sensor);
        }

        if (note) {
            robot.FeedStop();
        }
        robot.ShootStop(3);
        robot.IntakeStop(3);
        robot.ClimbStop(3);
        robot.Pivot(robot.pivot.mainMotor.getAbsoluteRawAngle(), 0.05);

        if (robot.readyToShoot) {
            robot.Shoot(0.5);
        }

        SmartDashboard.putNumber("sensor", sensor.getVoltage());
            
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean isNoteReady(AnalogInput sensor) {

        boolean isReady = false;

        isReady = sensor.getVoltage() < 0.01;

        return isReady;

    }

}
