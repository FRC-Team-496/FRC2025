// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;
  

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private int drivingCANId;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    this.drivingCANId = drivingCANId;
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    SparkMaxConfig drivingConfig = new SparkMaxConfig();

    drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    drivingConfig.encoder
        .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(ModuleConstants.kDrivingP,
              ModuleConstants.kDrivingI,
              ModuleConstants.kDrivingD,
              ModuleConstants.kDrivingFF)
        .outputRange(ModuleConstants.kDrivingMinOutput,
                    ModuleConstants.kDrivingMaxOutput);        
    m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig turningConfig = new SparkMaxConfig();
    turningConfig
        
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    turningConfig.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)

        .positionWrappingEnabled(true)
        .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
        .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)

        .pidf(ModuleConstants.kTurningP,
              ModuleConstants.kTurningI,
              ModuleConstants.kTurningD,
              ModuleConstants.kTurningFF)
        .outputRange(ModuleConstants.kTurningMinOutput,
                    ModuleConstants.kTurningMaxOutput);   


        //.pid(1.0, 0, 0);
    m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.

    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(); //removed "Type.kDutyCycle" from parameter








    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    // m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    // m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // // Apply position and velocity conversion factors for the driving encoder. The
    // // native units for position and velocity are rotations and RPM, respectively,
    // // but we want meters and meters per second to use with WPILib's swerve APIs.
    // m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    // m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // // Apply position and velocity conversion factors for the turning encoder. We
    // // want these in radians and radians per second to use with WPILib's swerve
    // // APIs.
    // m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    // m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // // the steering motor in the MAXSwerve Module.
    // m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // // Enable PID wrap around for the turning motor. This will allow the PID
    // // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // // to 10 degrees will go through 0 rather than the other direction which is a
    // // longer route.
    // m_turningPIDController.setPositionPIDWrappingEnabled(true);
    // m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    // m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    
    // // Set the PID gains for the driving motor. Note these are example gains, and you
    // // may need to tune them for your own robot!
    // m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    // m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    // m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    // m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    // m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
    //     ModuleConstants.kDrivingMaxOutput);

    // // Set the PID gains for the turning motor. Note these are example gains, and you
    // // may need to tune them for your own robot!
    // m_turningPIDController.setP(ModuleConstants.kTurningP);
    // m_turningPIDController.setI(ModuleConstants.kTurningI);
    // m_turningPIDController.setD(ModuleConstants.kTurningD);
    // m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    // m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
    //     ModuleConstants.kTurningMaxOutput);

    // m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    // m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    // m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    // m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    // // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // // operation, it will maintain the above configurations.
    // m_drivingSparkMax.burnFlash();
    // m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
    if(drivingCANId == DriveConstants.kFrontLeftDrivingCanId){
      SmartDashboard.putNumber("FL input Rot", desiredState.angle.getRadians());
      SmartDashboard.putNumber("Front Left cmd rot", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("Front Left Speed", optimizedDesiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Front Left angle", m_turningEncoder.getPosition());

    }
    if(drivingCANId == DriveConstants.kFrontRightDrivingCanId){
      SmartDashboard.putNumber("FR input Rot", desiredState.angle.getRadians());
      SmartDashboard.putNumber("Front Right Speed", optimizedDesiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Front Right Rotation", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("Front Right angle", m_turningEncoder.getPosition());

    }
    if(drivingCANId == DriveConstants.kRearLeftDrivingCanId){
      SmartDashboard.putNumber("BL input Rot", desiredState.angle.getRadians());
      SmartDashboard.putNumber("Back Left Speed", optimizedDesiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Back Left Rotation", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("Back Left angle", m_turningEncoder.getPosition());

    }
    if(drivingCANId == DriveConstants.kRearRightDrivingCanId){
      SmartDashboard.putNumber("BR input Rot", desiredState.angle.getRadians());
      SmartDashboard.putNumber("Back Right Speed", optimizedDesiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Back Right Rotation", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("Back Right angle", m_turningEncoder.getPosition());

    }
    m_desiredState = desiredState;

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
