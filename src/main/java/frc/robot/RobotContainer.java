// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Output;
import frc.robot.subsystems.SALUS;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Date;







/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Camera m_camera = new Camera();
  private final NavX m_gyro = new NavX();
  private final SALUS m_salus = new SALUS();
  private final Output m_output = new Output();
  private final Input m_input = new Input();
  private final Climbers m_climbers = new Climbers();
  private SendableChooser<Integer> m_chooser = new SendableChooser<Integer>(); 


//   private final Pixy2 m_pixy = new Pixy2();
  Thread m_visionThread;

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  //Axis[0] - left(-)/right(+)
  //Axis[1] - forward(-)/back(+)
  //Axis[2] - rotation (right is +)
  //Axis[3] altitude? up(-)/down(+)
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_driverController2 = new GenericHID(OIConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // m_pixy.init();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1), 0.2),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(0), 0.2),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(2), 0.2),
                false, (m_driverController.getRawAxis(3)+1)/2),
            m_robotDrive)
            );
        
      m_output.setDefaultCommand(new RunCommand(() -> m_output.stop(), m_output));

      m_input.setDefaultCommand(new RunCommand(() -> m_input.stop(), m_input));

        m_climbers.setDefaultCommand(new RunCommand(() -> m_climbers.stop(), m_climbers));        
        m_gyro.setDefaultCommand(
            new RunCommand(
            () -> m_gyro.putGyro()    
            , m_gyro)
        );
        m_camera.setDefaultCommand(
            new RunCommand(
            () -> m_camera.startCamera()    
            , m_camera)
        );

        //armStage
    
        // m_visionThread =
        // new Thread(
        //     () -> {
        //         m_camera.visionSystem();
        //     });
        //     m_visionThread.setDaemon(true);
        //     m_visionThread.start();
        }
        
    



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

   


  public class moveBack extends Command{
    double startX;
    DriveSubsystem m_robotDrive;
    private double distance; // in meters
    public moveBack(DriveSubsystem m_robotDrive, double distance){
      this.m_robotDrive = m_robotDrive;
      this.distance = distance;
    }


    @Override
    public void initialize() {
      startX = m_robotDrive.getPose().getX();
    }

    @Override
    public void execute() {
      m_robotDrive.drive(-.05, 0.0, 0.0, false, .3); //-.02
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_robotDrive.getPose().getX() - startX) > (distance / 1.31);  //.045   .9
    }

  }

  private Command getShootTime(double time, double speed){
    WaitCommand wait = new WaitCommand(time);
    RunCommand shoot = new RunCommand(() -> m_output.shoot(speed), m_output);
    InstantCommand stop = new InstantCommand(() -> m_output.stop(), m_output);
    RunCommand suck = new RunCommand(() -> m_input.suck(), m_input);
    InstantCommand stopSuck = new InstantCommand(() -> m_input.stop(), m_input);
    ParallelDeadlineGroup waitShoot = new ParallelDeadlineGroup(wait, shoot, suck);
    ParallelCommandGroup stopAll = new ParallelCommandGroup(stop, stopSuck);
    return new SequentialCommandGroup(waitShoot, stopAll);
  }






  private void configureButtonBindings() {

    
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, 1) //SALUS
    .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0, m_salus.calcX(), m_salus.calcYaw() / 2, false, 0.3), 
            m_robotDrive));

    new JoystickButton(m_driverController, 1) 
    .whileTrue(new InstantCommand(
            () -> m_salus.set()));

    new JoystickButton(m_driverController2, 2) // move back and shoot to small goal m_robotDrive, .045
      .whileTrue(new SequentialCommandGroup(new moveBack(m_robotDrive, .045), getShootTime(2.0, .22))); // new moveBack(m_robotDrive, .045)

    new JoystickButton(m_driverController2, 1) // move back and shoot to big goal, need to figure out keybinds
      .whileTrue(new SequentialCommandGroup(new moveBack(m_robotDrive, .61), getShootTime(2.0, .7)));

    new JoystickButton(m_driverController2, 5) //shoot
    .whileTrue(new RunCommand(
            () -> m_output.shoot(.7), m_output)); //speed set to the long goal speed since thats the one we may eyeball

    new JoystickButton(m_driverController2, 3) //suck
    .whileTrue(new RunCommand(
            () -> m_input.suck(), m_input));

    new JoystickButton(m_driverController2, 1)
    .whileFalse(new InstantCommand(
            () -> m_output.stop(), m_output));

    new JoystickButton(m_driverController2, 2)
    .whileFalse(new InstantCommand(
            () -> m_input.stop(), m_input));

    new JoystickButton(m_driverController, 5)
    .whileTrue(new RunCommand(
            () -> m_climbers.extend(1), m_climbers));
    new JoystickButton(m_driverController, 3)
    .whileTrue(new RunCommand(
            () -> m_climbers.extend(-1), m_climbers));
    // new JoystickButton(m_driverController, 3)
    // .whileFalse(new InstantCommand(
    //         () -> m_climbers.stop(), m_climbers));
    // new JoystickButton(m_driverController, 5)
    // .whileFalse(new InstantCommand(
    //         () -> m_climbers.stop(), m_climbers));

    
        
    
    
    m_chooser.addOption("Drop and slide", 3);
    SmartDashboard.putData(m_chooser);
    
    // new JoystickButton(m_driverController, 2).whileTrue(
    //     new InstantCommand(
    //         () -> m_pixy.setLamp(1, 1), 
    //         m_pixy));

    // new JoystickButton(m_driverController, 3).whileTrue(
    //     new InstantCommand(
    //         () -> m_pixy.setLamp(0, 0), 
    //         m_pixy));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // RunCommand auto = new RunCommand(new Autonomous(m_robotDrive, m_gyro));
    // return auto;
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, (m_driverController.getRawAxis(3)+1)/2 ));
   //}

  public void teloPeriodic(){
  }

  int state = 0;
  double mode;
  int intmode;
  double startTime = System.currentTimeMillis();
  private RunCommand m_forward = new RunCommand(() -> m_robotDrive.drive(.6, 0, 0, false, 0.3), m_robotDrive);
  private RunCommand m_backwards = new RunCommand(() -> m_robotDrive.drive(-.6, 0, 0, false, 0.3), m_robotDrive);
  private RunCommand m_lineUp = new RunCommand(() -> m_robotDrive.drive(0, m_salus.calcX(), m_salus.calcYaw() / 2, false, 0.3));
  private RunCommand m_shoot = new RunCommand(() -> m_output.shoot(.7));
  private RunCommand m_suck = new RunCommand(() -> m_input.suck());
  private RunCommand m_amp = new RunCommand(() -> new SequentialCommandGroup(new moveBack(m_robotDrive, .045), getShootTime(2.0, .22)));

  public void autoInnit(){
    state=0;
    mode = SmartDashboard.getNumber("Autonomous Mode", 1.0);
    System.out.println(mode);
    intmode = (int) mode;
    startTime = System.currentTimeMillis();
    m_salus.set();
  }



  public void autonomousPeriodic(){
    AHRS gyro = m_gyro.gyro();
    
    //commands
    //RunCommand forward = new RunCommand(() -> m_robotDrive.drive(.6, 0, 0, false, 0.3), m_robotDrive);
    
        switch(intmode) {
            case(1):
              m_backwards.schedule();
              if(System.currentTimeMillis() - startTime > 1000){
                CommandScheduler.getInstance().cancel(m_backwards);
              }
              break;
            case(2):
              m_lineUp.schedule();
              if(System.currentTimeMillis() - startTime > 1000 && System.currentTimeMillis() - startTime < 3000){
                CommandScheduler.getInstance().cancel(m_lineUp);
                m_suck.schedule();
                m_shoot.schedule();
              }
              if(System.currentTimeMillis() - startTime > 3000 && System.currentTimeMillis() - startTime < 4000){
                CommandScheduler.getInstance().cancel(m_suck);
                CommandScheduler.getInstance().cancel(m_shoot);
                m_backwards.schedule();
              }
              if(System.currentTimeMillis() - startTime > 4000){
                CommandScheduler.getInstance().cancel(m_backwards);
              }
              break;
            case(3):
              
              break;
        }
    }
}


