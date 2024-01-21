package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
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
        final int alignPreset = 4;

        //Operator Xbox Controller
        if (controller.getRightTriggerAxis() >= 0.5) {
            //shoot note
        }
        if (controller.getLeftTriggerAxis() >= 0.5) {
            //intake note
        }
        
        //Operator Button Board
        if (buttonBoard.getRawButton(ampPreset)) {
            //amp preset (118 or 119 degree rotation, low power)
        }
        if (buttonBoard.getRawButton(speakerPreset)) {
            //speaker preset (varying dgrees based on april tag, mid-full power based on range)
        }
        if (buttonBoard.getRawButton(intakePreset)) {
            //intake preset (20 degrees, low power)
        }
        if (buttonBoard.getRawButton(alignPreset)) {
            //align preset (align cannon for speaker based off april tags)
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
