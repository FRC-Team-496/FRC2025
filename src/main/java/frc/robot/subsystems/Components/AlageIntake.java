package frc.robot.subsystems.Components;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlageIntake extends SubsystemBase{
    double wheelSpeed = .4;
    double intakeRotationSpeed = .3;
    SparkMax m_wheels;
    SparkMax m_intake;

    SparkAbsoluteEncoder intakeMotorEncoder;
    double limit = .5;
    
    public AlageIntake(){
        m_wheels = new SparkMax(52, MotorType.kBrushless);
        m_intake = new SparkMax(62, MotorType.kBrushless);
        intakeMotorEncoder = m_intake.getAbsoluteEncoder();

    
        
        
    }

    public void spinAlageWheels(int dir){ // 1 for intake, -1 for push out
        m_wheels.set(wheelSpeed * dir);
    }

    public void moveAlageIntake(int dir){ // 1 for outward, -1 for backward (towards robot)
        m_intake.set(intakeRotationSpeed * dir);
    }

    public void intakeStop(){
        m_intake.set(0);
    }

    public void wheelsStop(){
        m_wheels.set(0);
    }

    public boolean checkSoftLimit(){
        double position = intakeMotorEncoder.getPosition();
        System.out.println(position);
        if(position > limit || position < 0){
            return false;
        }
        return true;
    }

    public double getPosition(){
        return intakeMotorEncoder.getPosition();
    }

}