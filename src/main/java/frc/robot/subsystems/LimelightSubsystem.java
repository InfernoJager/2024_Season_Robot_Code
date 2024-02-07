package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem {
    
    public LimelightSubsystem() {}

    public void LimelightWhere() {

        double[] params;

        // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        // NetworkTableEntry tx = table.getEntry("tx");
        // NetworkTableEntry ty = table.getEntry("ty");
        // NetworkTableEntry ta = table.getEntry("ta");

        // double x = tx.getDouble(0.0);
        // double y = ty.getDouble(0.0);
        // double area = ta.getDouble(0.0);
        // double z = 59 * Math.pow(area, -0.537);

        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightZ", z);
        // SmartDashboard.putNumber("LimelightArea", area);

        params = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        double meterToInch = 32;
        double x = params[0]*meterToInch;
        double y = params[1]*meterToInch;
        double z = params[2]*meterToInch;
        double roll = params[3];
        double pitch = params[4];
        double yaw = params[5];

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightZ", z);
        SmartDashboard.putNumber("LimelightRoll", roll);
        SmartDashboard.putNumber("LimelightPitch", pitch);
        SmartDashboard.putNumber("LimelightYaw", yaw);

    }

}
