package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.studica.frc.*;
import com.studica.frc.AHRS.NavXComType;


public class NavX extends SubsystemBase{

    AHRS navX;
    SPI port;
    public NavX(){
        //port = new SPI(Port.kMXP);   //Unused so I commented it out.
        navX = new AHRS(NavXComType.kMXP_SPI); //removed SPI.Port.kMXP from parameter, which port id do we want?

    }

    public AHRS gyro(){
        return navX;
    }
    
    public void putGyro(){
        SmartDashboard.putNumber("Pitch", navX.getRoll());
        SmartDashboard.putNumber("Roll", -navX.getPitch());
        SmartDashboard.putNumber("Yaw", navX.getYaw());
        SmartDashboard.putNumber("GYRO DISTANCE", Math.abs(navX.getDisplacementX() +navX.getDisplacementY() +navX.getDisplacementZ()));
    }

    public double pitch(){return navX.getRoll();}

    
}
