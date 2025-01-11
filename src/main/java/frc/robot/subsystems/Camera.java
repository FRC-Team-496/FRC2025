package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera extends SubsystemBase {
    private static double statX;
    private static double statYaw;
    private static double statY;


    public void startCamera() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry botpose = table.getEntry("botpose");
        NetworkTableEntry pipeline = table.getEntry("pipeline");

        //read values periodically
        double x = tx.getDouble(0.0);
        statX = x;
        double y = ty.getDouble(0.0);
        statY = y;
        double area = ta.getDouble(0.0);
        double[] bot = botpose.getDoubleArray(new double[6]);
        statYaw = bot[5];

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightBotX", bot[0]);
        SmartDashboard.putNumber("LimelightBotY", bot[1]);
        SmartDashboard.putNumber("LimelightBotZ", bot[2]);
        SmartDashboard.putNumber("zLimelightBotRoll", bot[3]);
        SmartDashboard.putNumber("LimelightBotPitch", bot[4]);
        SmartDashboard.putNumber("LimelightBotYaw", bot[5]);
        pipeline.setNumber(0);
    }

    public static double getX(){
        return statX;
    }

    public static double getYaw(){
        return statYaw;
    }

    public static double getY(){
        return statY;
    }
}
