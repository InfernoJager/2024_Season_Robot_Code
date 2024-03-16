package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class AutoDrive extends Command {
        private DriveSubsystem drive;
        private VectorR moveSpeed;
        private double position; 

        public AutoDrive(){
            this.drive = new DriveSubsystem();
        }

        @Override
        public void initialize(){

        }

        @Override
        public void execute() {
            moveSpeed.setMagnitude(0.15);
            moveSpeed.setAngle(0);
            drive.move(moveSpeed , 0, false, false);
            position = Math.abs(drive.modules.backRight.driveMotor.getEncoder().getPosition());
        }

        @Override
        public boolean isFinished() {
            if (position > 30) {
                return true;
            } else {
                return false;
            }
        }

        @Override
        public void end(boolean interrupted){
            drive.stop();
        }

}
