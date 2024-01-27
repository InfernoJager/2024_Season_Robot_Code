package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.swervemodule.SwerveModule;
import frc.robot.swervemodule.SwerveModules;

import java.lang.Math;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class DriverControls extends Command {
    
    private final DriveSubsystem drive;
    private final XboxController driverController;
    private final VectorR leftJoystick = new VectorR();

    public DriverControls(DriveSubsystem drive, XboxController driverController) {
        
        this.drive = drive;
        this.driverController = driverController;

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        boolean fastTurning = (driverController.getLeftTriggerAxis() >= 0.1 || driverController.getRightTriggerAxis() >= 0.1);
        boolean slowTurning = (driverController.getLeftBumper() || driverController.getRightBumper());
        boolean deadZone = (Math.abs(driverController.getLeftX()) <= 0.1 && Math.abs(driverController.getLeftY()) <= 0.1);

        SmartDashboard.putBoolean("deadzone", deadZone);
        SmartDashboard.putNumber("LeftX", driverController.getLeftX());
        SmartDashboard.putNumber("LeftY", driverController.getLeftY());

        //Driver Xbox Controller
        if (deadZone == false) {
            drive.move(driverController.getLeftX(), driverController.getLeftY(), driverController.getAButton(), driverController.getStartButton());
        } else {
            drive.stop();
        }
        if (fastTurning) {
            //fast spin
        }
        if (slowTurning) {
            //slow spin
        }
        if (driverController.getAButton()) {
            //slow down motors
        }
        if (driverController.getStartButton()) {
            //speed up motors
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    

}
