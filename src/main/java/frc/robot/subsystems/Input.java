package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Input extends SubsystemBase{

    CANSparkMax m_motor;
    private double speed = .2; //1
    DigitalInput m_limmitTop;
    DigitalInput m_limmitBottom;
    SparkMaxPIDController m_pidController;

    public Input(){

        m_motor = new CANSparkMax(62, MotorType.kBrushless);
        m_pidController = m_motor.getPIDController();

        m_limmitTop = new DigitalInput(0);
        m_limmitBottom = new DigitalInput(1);

    }

    // Encoder object created to display position values
    
   
    public void suck(){
        m_motor.set(speed);
    }

    public void stop(){
        m_motor.set(0);
    }
}
