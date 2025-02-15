package frc.robot.subsystems.Components;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMaxAlternateEncoder;


public class AlageIntake extends SubsystemBase{
    double wheelSpeed = .4;
    double intakeRotationSpeed = .25; //.3 max tested for lift
    SparkMax m_wheels;
    SparkMax m_intake;

    
    RelativeEncoder encoder;
    double limit = .5;
    
    public AlageIntake(){
        m_wheels = new SparkMax(52, MotorType.kBrushless);
        m_intake = new SparkMax(62, MotorType.kBrushless);

        encoder = m_intake.getEncoder();
        encoder.setPosition(0);
        // pid = new PIDController(.2, 0, .5);
        // intakeMotorConfig = new SparkMaxConfig();
        
        // encoder.setPosition(0);

        // intakeMotorConfig.softLimit
        //     .forwardSoftLimit(.5)
        //     .forwardSoftLimitEnabled(true);
    
        // m_intake.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        
        
    }

    public void spinAlageWheels(int dir){ // 1 for intake, -1 for push out
       
        m_wheels.set(wheelSpeed * dir);
    }

    public void moveAlageIntake(int dir){ // 1 for outward, -1 for backward (towards robot)
        SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());
        
        m_intake.get(); // do not delete
        m_intake.set(intakeRotationSpeed * dir);
        
    }

   
    

    public void intakeStop(){
        m_intake.set(0);
    }

    public void wheelsStop(){
        m_wheels.set(0);
    }

    

    

}