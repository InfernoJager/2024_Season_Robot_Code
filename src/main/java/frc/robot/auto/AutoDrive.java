package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class AutoDrive extends Command {
    private final DriveSubsystem drive;
    private VectorR moveSpeed;
    private double position;
    private double desiredPosition; 
    private double startPosition;
    private double angle;
    private double magnitude;

    public AutoDrive(DriveSubsystem drive, double distance, double angle, double magnitude) {

        // angle is on a cartesian, forward is 270, right is 0, back is 90, left is 180
        this.drive = drive;
        this.moveSpeed = new VectorR();
        this.desiredPosition = distance;
        this.angle = angle;
        this.magnitude = magnitude;

    }

    @Override
    public void initialize() {
        moveSpeed.setFromPolar(magnitude, angle);
        startPosition = drivePosition();
    }

    @Override
    public void execute() {
        drive.move(moveSpeed, 0, false, false, false);

        // One revolution is 0.5116 inches
        position = Math.abs(drivePosition() - startPosition);
        SmartDashboard.putNumber("Expected Position", position);
    }

    @Override
    public boolean isFinished() {
        if (position >= desiredPosition) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted){
        drive.stop();
    }

    private double drivePosition() {
        return drive.modules.backRight.driveMotor.getEncoder().getPosition();
    }

}
