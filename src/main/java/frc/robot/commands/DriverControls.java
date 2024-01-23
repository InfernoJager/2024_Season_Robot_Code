package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.Math;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;
import frc.robot.SwerveModules;
import frc.robot.SwerveModule;

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
        boolean deadZone = (driverController.getLeftX() <= Math.abs(0.1) || driverController.getLeftY() <= Math.abs(0.1));

        //Driver Xbox Controller
        if (deadZone == false) {
            drive.move(leftJoystick, fastTurning, slowTurning, 0.8);
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
