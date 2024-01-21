package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import java.lang.Math;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class Controls extends Command {
    
    private DriveSubsystem drive;
    private XboxController driverController = new XboxController(0);
    private XboxController operatorController = new XboxController(1);
    private GenericHID buttonBoard = new GenericHID(2);
    private VectorR leftJoystick = new VectorR();
    private VectorR rightJoystick = new VectorR();
    
    //Presets
    private int ampPreset = 1;
    private int speakerPreset = 2;
    private int intakePreset = 3;
    private int alignPreset = 4;

    public void controllers(DriveSubsystem drive) {
        
        this.drive = drive;

    }

    public void DriverControls() {
        
        boolean fastTurning = (driverController.getLeftTriggerAxis() >= 0.1 || driverController.getRightTriggerAxis() >= 0.1);
        boolean slowTurning = (driverController.getLeftBumper() || driverController.getRightBumper());
        boolean deadZone = (driverController.getLeftX() <= Math.abs(0.1) || driverController.getLeftY() <= Math.abs(0.1));

        //Driver Xbox Controller
        if (deadZone = false) {
            drive.move(leftJoystick, fastTurning, slowTurning);
        }
        if (fastTurning = true) {
            //fast spin
        }
        if (slowTurning = true) {
            //slow spin
        }
        if (driverController.getAButton()) {
            //slow down motors
        }
        if (driverController.getStartButton()) {
            //speed up motors
        }

    }

    public void OperatorControls() {
        
        //Operator Xbox Controller
        if (operatorController.getRightTriggerAxis() >= 0.5) {
            //shoot note
        }
        if (operatorController.getLeftTriggerAxis() >= 0.5) {
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

}