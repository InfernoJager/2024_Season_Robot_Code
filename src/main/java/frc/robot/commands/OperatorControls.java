package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import java.lang.Math;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;
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
        final int speakerPreset = 2;
        final int intakePreset = 3;
        final int intake = 4;
        final int speakerShoot = 5;
        final int ampShoot = 6;

        //Operator Xbox Controller
        // if (controller.getRightTriggerAxis() >= 0.5) {
        //     //shoot note speaker
        // }
        // if (controller.getRightTriggerAxis() >= 0.5 && controller.getAButton()) {
        //     //shoot note amp
        // }
        // if (controller.getLeftTriggerAxis() >= 0.5) {
        //     //intake note
        // }
        
        //Operator Button Board
        if (buttonBoard.getRawButtonPressed(ampPreset)) {
            //amp preset (118 or 119 degree rotation, low power)
        }
        if (buttonBoard.getRawButtonPressed(speakerPreset)) {
            //speaker preset (varying dgrees based on april tag, mid-full power based on range, no greater than 60, no less then 20)
        }
        if (buttonBoard.getRawButtonPressed(intakePreset)) {
            //intake preset (20 degrees, low power)
        }
        if (buttonBoard.getRawButton(intake)) {
            //intake note
        }
        if (buttonBoard.getRawButton(speakerShoot)) {
            //shoots note into speakers
        }
        if (buttonBoard.getRawButton(ampShoot)) {
            //shoots note into amp
        }
            

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}