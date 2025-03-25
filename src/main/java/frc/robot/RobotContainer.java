// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//fully importing com file odes not resolve object initialization errors; find com file and confirm it exists

//import edu.kauailabs.navx.frc.AHRS;
import com.studica.frc.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SALUS;
import frc.robot.subsystems.Components.Arm;
import frc.robot.subsystems.Components.AlageIntake;
import frc.robot.subsystems.Components.Climber;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;






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
  private final AlageIntake AlageIntake = new AlageIntake();
  private final Climber m_climbers = new Climber();
  private SendableChooser<Integer> m_chooser = new SendableChooser<Integer>(); 
  private final Arm m_arm = new Arm();

  private int autoScheduler = 0;


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
        

      

        //m_climbers.setDefaultCommand(new RunCommand(() -> m_climbers.stop(), m_climbers));        
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

   


   






  private void configureButtonBindings() {

    // Driving, joystick on driver controller
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


    // SALUS, button 1 driver controller (Back trigger)

    new JoystickButton(m_driverController, 3) //SALUS
    .whileTrue(new RunCommand(() -> m_robotDrive.drive(0, m_salus.calcX(), m_salus.calcYaw() / 2, false, 0.3)));

    // new JoystickButton(m_driverController, 3) //SALUS
    // .whileTrue(new SequentialCommandGroup(
    //   new lineUp(m_robotDrive),

    //   new moveStraight(m_robotDrive, Camera.getDistZ(), 1)

    // ));

            new JoystickButton(m_driverController, 3) //SALUS
    .whileTrue(new InstantCommand(
           () -> m_salus.set()));


   



    // Climbers, buttons 3 and 5 driver controller
    new JoystickButton(m_driverController, 2)
      .whileTrue(new InstantCommand(
             () -> m_climbers.extend(), m_climbers));

    new JoystickButton(m_driverController, 1)
     .whileTrue(new InstantCommand(
             () -> m_climbers.retract(), m_climbers));

    new JoystickButton(m_driverController, 2)
    .whileFalse(new InstantCommand(
            () -> m_climbers.stop(), m_climbers));
    new JoystickButton(m_driverController, 1)
    .whileFalse(new InstantCommand(
            () -> m_climbers.stop(), m_climbers));



            






           


        




            // new JoystickButton(m_driverController2, 7)
            // .whileTrue(new RunCommand(() -> m_arm.goToFeeder(), m_arm));

            

            new JoystickButton(m_driverController2, 6)
            .whileTrue(new RunCommand(() -> m_arm.goToPosition(2), m_arm));

            new JoystickButton(m_driverController2, 5)
            .whileTrue(new RunCommand(() -> m_arm.goToPosition(3), m_arm));

            



            new JoystickButton(m_driverController2, 6)
            .whileFalse(new RunCommand(() -> m_arm.dropFrom(2), m_arm));

            new JoystickButton(m_driverController2, 5)
            .whileFalse(new RunCommand(() -> m_arm.dropFrom(3), m_arm));

            


            new JoystickButton(m_driverController2, 8)
            .whileTrue(new RunCommand(() -> m_arm.resetArm(), m_arm));
            



            // new JoystickButton(m_driverController, 3) // TEST WIT APRIL TAG
            // .whileTrue(scoreCoral(1, 3));

            new JoystickButton(m_driverController2, 7) // TEST WIT APRIL TAG
            .whileTrue(new RunCommand(() -> m_arm.goToFeeder(), m_arm)

              
            );            

        
            

            
       
        //Algae Intake commands
                // new JoystickButton(m_driverController, 7)
                // .whileTrue(new InstantCommand(
                //         () -> AlageIntake.wheelsStop(), AlageIntake));
                
                // new JoystickButton(m_driverController, 9)
                // .whileTrue(new InstantCommand(
                //         () -> AlageIntake.spinAlageWheels(-1), AlageIntake));

                

                new JoystickButton(m_driverController2, 1)
                .onTrue(new RunCommand(
                        () -> AlageIntake.setActive(), AlageIntake));

                new JoystickButton(m_driverController2, 2)
                .onTrue(new RunCommand(
                        () -> AlageIntake.setIdle(), AlageIntake));

                new JoystickButton(m_driverController2, 3)
                .whileTrue(new InstantCommand(
                        () -> AlageIntake.wheelSequence(), AlageIntake));

                        new JoystickButton(m_driverController2, 3)
                .whileTrue(new RunCommand(
                        () -> AlageIntake.lockMotors(), AlageIntake));

       


                
                
                        


  }


  




  public class moveStraight extends Command{
    double startX;
    DriveSubsystem m_robotDrive;
    private double distance; // in meters

    private int direction;

    public moveStraight(DriveSubsystem m_robotDrive, double distance, int direction){
      this.m_robotDrive = m_robotDrive;
      this.distance = distance;
      this.direction = direction;
    }

    @Override
    public void initialize() {
      startX = m_robotDrive.getPose().getX();
    }

    @Override
    public void execute() {
      m_robotDrive.drive(.6 * direction, 0.0, 0.0, false, .3); //-.02
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_robotDrive.getPose().getX() - startX) > (distance / 1.31);  
    }

  }


  public class moveSide extends Command{
    double startY;
    DriveSubsystem m_robotDrive;
    private double distance; // in meters

    private int direction; // 1 for right?

    public moveSide(DriveSubsystem m_robotDrive, double distance, int direction){
      this.m_robotDrive = m_robotDrive;
      this.distance = distance;
      this.direction = direction;
    }

    @Override
    public void initialize() {
      startY = m_robotDrive.getPose().getY();
    }

    @Override
    public void execute() {
      m_robotDrive.drive(0.0, .3 * direction, 0.0, false, .3); //-.02
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_robotDrive.getPose().getY() - startY) > (distance / 1.31);  //.045   .9
    }

  }


  public class turn extends Command{
    double yawDiff;
    DriveSubsystem m_robotDrive;
    private double angle; // in radians (maybe?)
    double startingYaw;
    private int direction; // 1 for right?


    // 1 for clockwise, -1 for counter clockwise? (maybe?)

    public turn(DriveSubsystem m_robotDrive, double angle, int direction){
      this.m_robotDrive = m_robotDrive;
      this.angle = angle;
      this.direction = direction;
 
    }

    @Override
    public void initialize() {
      startingYaw = Camera.getYaw();
    }

    @Override
    public void execute() {
      
      m_robotDrive.drive(0.0, 0.0, 2 * direction, false, .3); // maybe add field relative true?
      yawDiff = Math.abs(Camera.getYaw() - startingYaw);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(yawDiff - angle) < (1);  //add directional component
    }

  }



















  public class moveArm extends Command{
    
    private Arm arm;
    private int level;
    


    public moveArm(Arm arm, int level){
      addRequirements(arm);
      this.arm = arm;
      this.level = level;
    }

    

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      arm.goToPosition(level);
    }

    @Override
    public boolean isFinished() {
      return arm.checkConditions(level);
    }

  }

  public class intake extends Command{
    
    private Arm arm;
    


    public intake(Arm arm){
      addRequirements(arm);
      this.arm = arm;
    }

    

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      arm.goToFeeder();
    }

    @Override
    public boolean isFinished() {
      return arm.checkFeederConditions();
    }

  }

  public class dropArm extends Command{
    
    private Arm arm;
    private int level;
    

    public dropArm(Arm arm, int level){
      addRequirements(arm);
      this.arm = arm;
      this.level = level;
    }
    

    @Override
    public void execute() {
      arm.dropFrom(level);
    }

    @Override
    public boolean isFinished() {
      return arm.checkDrop(level);
    }

  }


  


  // 1 for right side -1 for left side
  public SequentialCommandGroup scoreCoral(int side, int level){
    return new SequentialCommandGroup(
        
    // align to tag

        //alignToTag(),

        new lineUp(m_robotDrive),

        new moveStraight(m_robotDrive, Camera.getDistZ(), (int) Math.signum(Camera.getDistZ())),



        new WaitCommand(1.0),

        new moveSide(m_robotDrive, .1651, 1),

        new WaitCommand(1.0),

        new moveArm(m_arm, level),

        new WaitCommand(1.0),

        new moveStraight(m_robotDrive, .848, 1),  

        new WaitCommand(1.0),

        new dropArm(m_arm, level)

    );
  }


  public class lineUp extends Command{
    double startY;
    DriveSubsystem m_robotDrive;



    public lineUp(DriveSubsystem m_robotDrive){
      this.m_robotDrive = m_robotDrive;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
      m_robotDrive.drive(0, m_salus.calcX(), m_salus.calcYaw() / 2, false, 0.3);
    }

    @Override
    public boolean isFinished() {
        return Camera.getDistX() < .02 && Camera.getDistYaw() < 1 && Camera.getDistYaw() > -1 ;  //.045   .9
    }

  }


  // public SequentialCommandGroup alignToTag(){
  //       return new SequentialCommandGroup(

  //           new moveSide(m_robotDrive, Camera.getDistX(), (int) MathUtil.clamp(Camera.getDistX(), -1, 1)),
            
  //           new moveStraight(m_robotDrive, Camera.getDistY() - 1, (int) MathUtil.clamp(Camera.getDistY() - 1, -1, 1))

  //       );  
    
    
  // }


  


  



  

   

  


  

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
  // private RunCommand m_forward = new RunCommand(() -> m_robotDrive.drive(.6, 0, 0, false, 0.3), m_robotDrive);
  // private RunCommand m_backwards = new RunCommand(() -> m_robotDrive.drive(-.6, 0, 0, false, 0.3), m_robotDrive);
  // private RunCommand m_lineUp = new RunCommand(() -> m_robotDrive.drive(0, m_salus.calcX(), m_salus.calcYaw() / 2, false, 0.3));

  public void teleopInit(){
        
  }

  public void autoInnit(){
    state=0;
    mode = SmartDashboard.getNumber("Autonomous Mode", 1.0);
    System.out.println(mode);
    intmode = (int) mode;
    startTime = System.currentTimeMillis();

    // SequentialCommandGroup case1 = new SequentialCommandGroup(
    // new moveStraight(m_robotDrive, 1, 1),
    // new WaitCommand(3.0),
    // scoreCoral(1, 3) // 3 is level 4
    // );

    //m_robotDrive.drive(0, 1, 0, false, .3);

    
  }



  public void autonomousPeriodic(){
    
    // RunCommand forward = new RunCommand(() -> m_robotDrive.drive(.4, 0, 0, false, 0.3), m_robotDrive);
    
    // forward.schedule();
    // if(System.currentTimeMillis() - startTime > 3000){
    //   CommandScheduler.getInstance().cancel(forward);
    // }

    RunCommand forward = new RunCommand(() -> m_robotDrive.drive(.4, 0, 0, false, 0.3), m_robotDrive);
    

    Command moveArm = new moveArm(m_arm, 2);
    
    Command dropArm = new dropArm(m_arm, 2);

    moveArm.schedule();
    forward.schedule();
      
    
  
    if(System.currentTimeMillis() - startTime > 2000){
      CommandScheduler.getInstance().cancel(forward);
      
      
    }

    if(System.currentTimeMillis() - startTime > 4000){
      dropArm.schedule();
    }

    // if(System.currentTimeMillis() - startTime > 5000){
    //   dropArm.schedule();
    // }

    //commands
    // RunCommand forward = new RunCommand(() -> m_robotDrive.drive(.6, 0, 0, false, 0.3), m_robotDrive);
    // SequentialCommandGroup case1 = new SequentialCommandGroup(
    // new moveStraight(m_robotDrive, .3, 1),
    // new WaitCommand(3.0),
    // scoreCoral(1, 3) // 3 is level 4
    // );

    //     switch(intmode) {
    //         case(1):
    //         if(!case1.isScheduled()){
    //           case1.schedule();
    //         }
              
              
    //           break;
    //         case(2):
              
    //           new SequentialCommandGroup(

    //           new moveStraight(m_robotDrive, 1.524, 1), //wy 1.5?? this is wrong

    //           new turn(m_robotDrive, 150 * Math.PI / 180, 1),

    //           scoreCoral(1, 3),

    //           new moveStraight(m_robotDrive, 1, -1),

    //           new turn(m_robotDrive, Math.PI, 1),

    //           // feeder
    //           alignToTag(),

    //           new moveStraight(m_robotDrive, .7, 1) //UNFINISHED

              



    //           ).schedule();

              
              
    //           break;

    //     }
    
}
}


