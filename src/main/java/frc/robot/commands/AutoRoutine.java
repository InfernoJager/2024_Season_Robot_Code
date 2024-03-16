package frc.robot.commands;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoShoot;

public class AutoRoutine {
    
    private final AutoDrive drive;
    private final AutoShoot shoot;

    public AutoRoutine() {

        this.drive = new AutoDrive();
        this.shoot = new AutoShoot();

    }

    public void TwoNoteAuto() {

        drive.execute();

    }

}
