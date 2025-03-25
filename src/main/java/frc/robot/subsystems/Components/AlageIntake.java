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

import edu.wpi.first.math.MathUtil;

public class AlageIntake extends SubsystemBase{
    double wheelSpeed = .4;
    double intakeRotationSpeed = .1; //.3 max tested for lift
    SparkMax m_wheels;
    SparkMax m_intake;
    PIDController pid;


    double activePos = -22;

    double idlePos = -1; //?
    
    RelativeEncoder encoder;
    RelativeEncoder wheelEncoder;

    double wheelsLockPos;

    int wheelSequence = 0;

    // RelativeEncoder tempEncoder;
    
    
    public AlageIntake(){
        m_wheels = new SparkMax(52, MotorType.kBrushless);
        m_intake = new SparkMax(62, MotorType.kBrushless);

        encoder = m_intake.getEncoder();
        wheelEncoder = m_wheels.getEncoder();

        wheelEncoder.setPosition(0);
        encoder.setPosition(0);

        SmartDashboard.putNumber("starting pos",encoder.getPosition());

        
       
        pid = new PIDController(.3, 0, 0);  // .3, .5, .3

        
        
    }

    

    




    public void setActive(){
        
     
        SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());
        

        m_intake.set(MathUtil.clamp(pid.calculate(encoder.getPosition(), activePos), -.4, .4)); //-35
    }


    public void setIdle(){
        
        
        SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());

        m_intake.set(MathUtil.clamp(pid.calculate(encoder.getPosition(), idlePos), -.2, .2)); //-35
    }


    public void wheelSequence(){
        if(wheelSequence == 0){
            m_wheels.set(-.7);
        }
        
        else if(wheelSequence == 1){
            getLockPosition();
        }
        else if(wheelSequence == 2){
            m_wheels.set(.4);
        }
        else if(wheelSequence == 3){
            m_wheels.set(0);
        }

        wheelSequence = (wheelSequence + 1) % 4;
    }




    public void spinAlageWheels(int dir){ // 1 for intake, -1 for push out
       
        m_wheels.set(wheelSpeed * dir);
    }

    public void wheelsStop(){
        m_wheels.set(0);
    }


    public void getLockPosition(){
        wheelsLockPos = wheelEncoder.getPosition();
    }

    public void lockMotors(){ // IDKI IF THIS WORKS
        if(wheelSequence == 2){
            m_wheels.set(MathUtil.clamp(pid.calculate(wheelEncoder.getPosition(), wheelsLockPos - 10), -.2, .2));
            


        }
    }

    

    

}