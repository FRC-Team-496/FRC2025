package frc.robot.subsystems.Components;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


public class Arm extends SubsystemBase{
    
    SparkMax armMotor;
    SparkMax clawMotor;

    double armSpeed = .3;
    
    double clawSpeed = .15;
    

    double[] coralTreeArmPositions = {0.0, 0.0, 0.0, 40.0};

    double feederArmPos = 80;

//
    // add to go to level method
    double[] coralTreeClawPositions = {1.0, 2.0, 3.0, 4.0}; 

    double feederClawPos = 3.5;
    

    double forwardLimit = 100;
    double reverseLimit = 0;

    RelativeEncoder armEncoder;
    RelativeEncoder clawEncoder;

    PIDController armPid;
    PIDController clawPid;

    public Arm(){
        
        armMotor = new SparkMax(51, MotorType.kBrushless);
        clawMotor = new SparkMax(53, MotorType.kBrushless);

        armEncoder = armMotor.getEncoder();


        clawEncoder = armMotor.getEncoder();

        resetEncoder();


        armPid = new PIDController(.4, 0, 0);
        clawPid = new PIDController(.4, 0, 0);

    

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.softLimit.forwardSoftLimit(forwardLimit);
        armConfig.softLimit.reverseSoftLimit(reverseLimit);
        armConfig.softLimit.forwardSoftLimitEnabled(true);
        armConfig.softLimit.reverseSoftLimitEnabled(true);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    
    public void goToFeeder(){
        //armMotor.set(MathUtil.clamp(armPid.calculate(armEncoder.getPosition(), feederArmPos), -.2, .2)); 

        clawMotor.set(MathUtil.clamp(clawPid.calculate(clawEncoder.getPosition(), feederClawPos), -.2, .2)); 


        SmartDashboard.putNumber("clawPostion", clawEncoder.getPosition());
        SmartDashboard.putNumber("armPostion", armEncoder.getPosition());

        System.out.println(clawEncoder.getPosition());
    }






    public void resetEncoder(){
        clawEncoder.setPosition(0.0);
        armEncoder.setPosition(0.0);

        SmartDashboard.putNumber("clawPostion", clawEncoder.getPosition());
        SmartDashboard.putNumber("armPostion", armEncoder.getPosition());

    }

}