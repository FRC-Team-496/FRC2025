package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;

//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbers extends SubsystemBase{
    CANSparkMax m_motor1;
    CANSparkMax m_motor2;
    double speed = .4;
    
    public Climbers(){
        m_motor1 = new CANSparkMax(20, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(21, MotorType.kBrushless);
        //m_motor1.getEncoder().setPosition(0.0);
        //m_motor2.getEncoder().setPosition(0.0);
        // -178
    }

    // Encoder object created to display position values
    
   
    public void extend(int dir){
        m_motor1.set(dir*speed);
        m_motor2.set(dir*speed);
    }

    public void stop(){
        m_motor1.set(0);
        m_motor2.set(0);
    }
}
