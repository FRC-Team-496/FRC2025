package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera extends SubsystemBase {
    private static double statX;
    private static double statYaw;
    private static double statY;
    private static int detected_ID;

    private static int cameraPipelineID = 0;
    private static int neuralNetworkpipelineId;

    public void startCamera() {
        NetworkTable armTable = NetworkTableInstance.getDefault().getTable("limelight-arm");
        NetworkTableEntry tx = armTable.getEntry("tx");
        NetworkTableEntry ty = armTable.getEntry("ty");
        NetworkTableEntry ta = armTable.getEntry("ta");
        NetworkTableEntry botpose = armTable.getEntry("botpose");
        NetworkTableEntry pipeline = armTable.getEntry("pipeline");
        int aprilTagID =  (int) armTable.getEntry("tid").getDouble(-1);
        detected_ID = aprilTagID;
        
        
        //read values periodically
        double x = tx.getDouble(0.0);
        statX = x;
        double y = ty.getDouble(0.0);
        statY = y;
        double area = ta.getDouble(0.0);
        double[] bot = botpose.getDoubleArray(new double[6]);
        statYaw = bot[5];

        //System.out.println(getDetected_ID() == 7);
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
        pipeline.setNumber(cameraPipelineID);
        
        SmartDashboard.putNumber("Tag ID", aprilTagID);


        NetworkTable algaeTable = NetworkTableInstance.getDefault().getTable("limelight-algae");
        SmartDashboard.putNumber("LimelightXAlgae", algaeTable.getEntry("tx").getDouble(10.0));
    }


    public static int getDetected_ID(){
        //System.out.println(detected_ID);
        return detected_ID;
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
