package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem {
    
    public LimelightSubsystem() {}

    public void LimelightWhere() {

        // double[] params;

        // params = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        // /*39.37 inches in a meter, but limelight calibration required 32*/
        // double meterToInch = 32;
        // double x = Math.round(params[0]*meterToInch);
        // double y = Math.round(params[1]*meterToInch);
        // double z = Math.round(params[2]*meterToInch);
        // double roll = params[3];
        // double pitch = params[4];
        // double yaw = params[5];

        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightZ", z);
        // SmartDashboard.putNumber("LimelightRoll", roll);
        // SmartDashboard.putNumber("LimelightPitch", pitch);
        // SmartDashboard.putNumber("LimelightYaw", yaw);

    }

}
