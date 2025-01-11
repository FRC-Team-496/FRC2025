package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX extends SubsystemBase{

    AHRS navX;

    public NavX(){

        navX = new AHRS(SPI.Port.kMXP);

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
