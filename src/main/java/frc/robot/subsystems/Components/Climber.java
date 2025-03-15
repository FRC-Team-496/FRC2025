package frc.robot.subsystems.Components;


import com.ctre.phoenix6.hardware.*;


//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;

//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    //SparkMax m_motor1;
    //SparkMax m_motor2;

    TalonFX m_kraken;
    
    
    public Climber(){
        //m_motor1 = new SparkMax(20, MotorType.kBrushless);
        //m_motor2 = new SparkMax(21, MotorType.kBrushless);
        //m_motor1.getEncoder().setPosition(0.0);
        //m_motor2.getEncoder().setPosition(0.0);
     
        m_kraken = new TalonFX(0);
    }

    // Encoder object created to display position values
    
   
    public void retract(){
         m_kraken.set(.4);
    }

    public void extend(){
        m_kraken.set(-.6);
   }

    public void stop(){
         m_kraken.set(0);
         
    }
}
