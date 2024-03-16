package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class AutoDrive extends Command {
        private final DriveSubsystem drive;
        private VectorR moveSpeed;
        private double position; 

        public AutoDrive(DriveSubsystem drive) {

            this.drive = drive;
            this.moveSpeed = new VectorR();

        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {
            drive.move(Move(0.15, 0), 0, false, false);
            position = Math.abs(drive.modules.backRight.driveMotor.getEncoder().getPosition());
        }

        @Override
        public boolean isFinished() {
            if (position > 500) {
                SmartDashboard.putBoolean("AutoFinished", true);
                return true;
            } else {
                return false;
            }
        }

        @Override
        public void end(boolean interrupted){
            drive.stop();
        }

        private VectorR Move(double magnitude, double angle) {

            moveSpeed.setMagnitude(magnitude);
            moveSpeed.setAngle(angle);

            return moveSpeed;

        }

}
