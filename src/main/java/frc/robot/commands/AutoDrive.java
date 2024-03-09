package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.VectorR;

public class AutoDrive extends Command {
        private DriveSubsystem drive;
        private VectorR moveSpeed; 

        public AutoDrive(DriveSubsystem drive){
            this.drive = drive;
        }

        @Override
        public void initialize(){

        }

        @Override
        public void execute(){
            moveSpeed.setMagnitude(0.1);
            moveSpeed.setAngle(0);
            drive.move(moveSpeed , 0, false, false);
        }

        @Override
        public boolean isFinished() {
          return true;
        }

        @Override
        public void end(boolean interrupted){
            moveSpeed.setMagnitude(0.0);
            drive.move(moveSpeed, 0, false, false);
        }

}
