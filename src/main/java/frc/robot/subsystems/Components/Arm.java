package frc.robot.subsystems.Components;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMaxAlternateEncoder;


public class Arm extends SubsystemBase{
    
    SparkMax armMotor;
    
    double speed = .3;
    
    public Arm(){
        
        armMotor = new SparkMax(21, MotorType.kBrushless);
        


        
        
    }

    public void moveArm(int dir){ // 1 for intake, -1 for push out
        armMotor.set(speed * dir);
    }

    public void wheelsStop(){
        armMotor.set(0);
    }

    

    

}