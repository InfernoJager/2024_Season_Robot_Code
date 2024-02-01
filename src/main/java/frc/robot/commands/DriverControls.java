package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;
import frc.robot.utils.VectorR;

public class DriverControls extends Command {
    
    private final DriveSubsystem drive;
    private final XboxController driverController;
    private VectorR leftJoystick = new VectorR();

    public DriverControls(DriveSubsystem drive, XboxController driverController) {
        
        this.drive = drive;
        this.driverController = driverController;

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        boolean deadZone = (Math.abs(driverController.getLeftX()) <= 0.1 && Math.abs(driverController.getLeftY()) <= 0.1);
        if (!deadZone) {
            leftJoystick = VectorR.fromCartesian(driverController.getLeftX()*0.2, driverController.getLeftY()*0.2);
        } else {
            leftJoystick = new VectorR();
        }
        // boolean fastTurning = (driverController.getLeftTriggerAxis() >= 0.1 || driverController.getRightTriggerAxis() >= 0.1);
        // boolean slowTurning = (driverController.getLeftBumper() || driverController.getRightBumper());

        SmartDashboard.putBoolean("deadzone", deadZone);
        SmartDashboard.putNumber("LeftX", driverController.getLeftX());
        SmartDashboard.putNumber("LeftY", driverController.getLeftY());

        //Driver Xbox Controller
        // if (deadZone == false) {
        //     drive.move(driverController.getLeftX(), driverController.getLeftY(), driverController.getAButton(), driverController.getStartButton());
        // } else {
        //     drive.stop();
        // }
        if (driverController.getRightTriggerAxis() > 0.1) {
            drive.move(leftJoystick, 0.25, driverController.getAButton(), driverController.getStartButton());
        } else if (driverController.getLeftTriggerAxis() > 0.1) {
            drive.move(leftJoystick, -0.25, driverController.getAButton(), driverController.getStartButton());
        } else if (driverController.getRightBumper()) {
            drive.move(leftJoystick, 0.125, driverController.getAButton(), driverController.getStartButton());
        } else if (driverController.getLeftBumper()) {
            drive.move(leftJoystick, -0.125, driverController.getAButton(), driverController.getStartButton());
        } else if (leftJoystick.getMagnitude() > 0.1) {
            drive.move(leftJoystick, 0, driverController.getAButton(), driverController.getStartButton());
        } else {
            drive.stop();
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    

}
