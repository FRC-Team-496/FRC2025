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
    

    int count = 0;

    SparkMax armMotor = new SparkMax(51, MotorType.kBrushless);
    SparkMax clawMotor = new SparkMax(53, MotorType.kBrushless);

    double armSpeed = .3;
    
    double clawSpeed = .15;
    

    double[] coralTreeArmPositions = {0.0, 20.0, 0.0, 130.0};

    double feederArmPos = 83;

//
    // add to go to level method
    double[] coralTreeClawPositions = {0.0, 4.0, 14, 15}; 

    double feederClawPos = 3.5;
    

    double forwardLimit = 137;
    double reverseLimit = 0;

    RelativeEncoder armEncoder;
    RelativeEncoder clawEncoder;

    PIDController armPid;
    PIDController clawPid;

    public Arm(){
        


        armEncoder = armMotor.getEncoder();
        clawEncoder = clawMotor.getEncoder();

        


        armPid = new PIDController(.4, 0, 0);
        clawPid = new PIDController(.4, 0, 0);

    

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.softLimit.forwardSoftLimit(forwardLimit);
        armConfig.softLimit.reverseSoftLimit(reverseLimit);
        armConfig.softLimit.forwardSoftLimitEnabled(true);
        armConfig.softLimit.reverseSoftLimitEnabled(true);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig clawConfig = new SparkMaxConfig();

        clawConfig.softLimit.reverseSoftLimit(0);
        clawConfig.softLimit.reverseSoftLimitEnabled(true);
        // clawConfig.softLimit.forwardSoftLimit(?);
        // clawConfig.softLimit.forwardSoftLimitEnabled(true);
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        armEncoder.setPosition(0);
        clawEncoder.setPosition(0);

        SmartDashboard.putNumber("clawPostion", clawEncoder.getPosition());
        SmartDashboard.putNumber("armPostion", armEncoder.getPosition());

    }

    
    public void goToFeeder(){
        armMotor.set(MathUtil.clamp(armPid.calculate(armEncoder.getPosition(), feederArmPos), -.3, .3)); 

        clawMotor.set(MathUtil.clamp(clawPid.calculate(clawEncoder.getPosition(), feederClawPos), -.2, .2)); 


        SmartDashboard.putNumber("clawPostion", clawEncoder.getPosition());
        SmartDashboard.putNumber("armPostion", armEncoder.getPosition());

    }



    public void goToPosition(int level){
        if((armEncoder.getPosition() > coralTreeArmPositions[level] - 3) && (armEncoder.getPosition() < coralTreeArmPositions[level] + 3)){
            clawMotor.set(MathUtil.clamp(clawPid.calculate(clawEncoder.getPosition(), coralTreeClawPositions[level]), -.15, .15)); 

        }
      
        
        armMotor.set(MathUtil.clamp(armPid.calculate(armEncoder.getPosition(), coralTreeArmPositions[level]), -.3, .3)); 


        SmartDashboard.putNumber("clawPostion", clawEncoder.getPosition());
        SmartDashboard.putNumber("armPostion", armEncoder.getPosition());

    }

    public boolean checkConditions(int level){
        if(Math.abs(armEncoder.getPosition() - coralTreeArmPositions[level]) < 1 && Math.abs(clawEncoder.getPosition() - coralTreeClawPositions[level]) < 1){
            return true;
        }
        
        return false;
    }

    public boolean checkFeederConditions(){
        if(Math.abs(armEncoder.getPosition() - feederArmPos) == 0 && Math.abs(clawEncoder.getPosition() - feederClawPos) == 0){
            return true;
        }
        
        return false;
    }

    public boolean checkDrop(int level){
        if(Math.abs(clawEncoder.getPosition() - (coralTreeClawPositions[level] - 3)) < 1){
            return true;
        }
        
        return false;
    }

    public void dropFrom(int level){

        clawMotor.set(MathUtil.clamp(clawPid.calculate(clawEncoder.getPosition(), coralTreeClawPositions[level] - 3), -.1, .1));  // -2


        SmartDashboard.putNumber("clawPostion", clawEncoder.getPosition());
    }


    public void resetArm(){
        clawMotor.set(MathUtil.clamp(clawPid.calculate(clawEncoder.getPosition(), 0), -.2, .2));  // -2
        armMotor.set(MathUtil.clamp(armPid.calculate(armEncoder.getPosition(), 0), -.3, .3));  // -2


    }



    public void armUp(){
        armMotor.set(1);
    }

    public void armDown(){
        armMotor.set(-1);
    }

    public void stopArm(){
        armMotor.set(0);
    }

    public void resetEncoder(){
        
            clawEncoder.setPosition(0.0);
            armEncoder.setPosition(0.0);

            SmartDashboard.putNumber("clawPostion", clawEncoder.getPosition());
            SmartDashboard.putNumber("armPostion", armEncoder.getPosition());

    }

}