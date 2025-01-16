package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Input extends SubsystemBase{

    SparkMax m_motor;
    private double speed = .2; //1
    DigitalInput m_limmitTop;
    DigitalInput m_limmitBottom;
    SparkClosedLoopController m_pidController;

    public Input(){

        m_motor = new SparkMax(62, MotorType.kBrushless);
        m_pidController = m_motor.getClosedLoopController();

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
