// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Java Stuff
import java.util.List;
import java.util.HashMap;
import java.util.ArrayList;

//WpiLib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

//CTRE
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


//Constants
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

//Miscellaneous Commands
import frc.robot.commands.SetCoast;
import frc.robot.commands.SetBrake;
import frc.robot.commands.DoTimeOut;

//Arm Commands
import frc.robot.commands.Arm.ArmExtend;
import frc.robot.commands.Arm.ArmRetract;

//AutoCommands
import frc.robot.commands.Auto.AutoDriveFor;
import frc.robot.commands.Auto.ChargeStationLevel;
import frc.robot.commands.Auto.PrototypeChargeLeveling;

//Claw Commands
import frc.robot.commands.Claw.ClawClose;
import frc.robot.commands.Claw.ClawOpen;

//Lights Commands
import frc.robot.commands.Lights.Flashy;
import frc.robot.commands.Lights.SolidPurple;
import frc.robot.commands.Lights.SolidYellow;

//WristCommands
import frc.robot.commands.Wrist.WristToStart;
import frc.robot.commands.Wrist.WristToDrive;
import frc.robot.commands.Wrist.WristToBottom;
import frc.robot.commands.Wrist.WristToTopPeg;
import frc.robot.commands.Wrist.WristToTopShelf;
import frc.robot.commands.Wrist.WristToMiddlePeg;
import frc.robot.commands.Wrist.WristToMiddleShelf;
import frc.robot.commands.Wrist.WristToFeederStation;

//Enums
import frc.robot.enums.AutoBalanceDirection;


//subsystems
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.DriveSubsystem;







/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  

  // The driver's controller
  Joystick m_driveJoystick = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_actuatorJoystick = new Joystick(OIConstants.kActuatorControllerPort);

  // subsystems
  private final Claw m_claw = new Claw();
  private final Wrist m_wrist = new Wrist();
  private final Arm m_arm = new Arm();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final Lights m_lights = new Lights();


  // commands 
  private final ClawClose m_clawClose = new ClawClose(m_claw);
  private final ClawOpen m_clawOpen = new ClawOpen(m_claw);
  private final WristToBottom m_wristToBottom = new WristToBottom(m_wrist);
  private final WristToFeederStation m_wristToFeederStation = new WristToFeederStation(m_wrist);
  private final WristToMiddlePeg m_wristToMiddlePeg = new WristToMiddlePeg(m_wrist);
  private final WristToMiddleShelf m_wristToMiddleShelf = new WristToMiddleShelf(m_wrist);
  private final WristToTopPeg m_wristToTopPeg = new WristToTopPeg(m_wrist);
  private final WristToTopShelf m_wristToTopShelf = new WristToTopShelf(m_wrist);
  private final WristToStart m_wristToStart = new WristToStart(m_wrist);
  private final ArmExtend m_armExtend = new ArmExtend(m_arm);
  private final ArmRetract m_armRetract = new ArmRetract(m_arm);
  private final WristToDrive m_wristToDrive = new WristToDrive(m_wrist);
  private final SetBrake m_setBrake = new SetBrake(m_driveSubsystem);
  private final SetCoast m_setCoast = new SetCoast(m_driveSubsystem);
  private final SolidPurple m_solidPurple = new SolidPurple(m_lights);
  private final SolidYellow m_solidYellow = new SolidYellow(m_lights);
  private final Flashy m_flashy = new Flashy(m_lights);


  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  HashMap<String, Command> autoMap = new HashMap<>();
  private final ArrayList<TalonFX> m_talons = m_wrist.getMotors();
  private final Orchestra m_orchestra = new Orchestra(m_talons);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_orchestra.loadMusic("MissionImpossible.chrp");
    m_orchestra.play();
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driveJoystick.getY(), OIConstants.kXDriveDeadband),
                -MathUtil.applyDeadband(m_driveJoystick.getX(), OIConstants.kYDriveDeadband),
                -MathUtil.applyDeadband(m_driveJoystick.getZ(), OIConstants.kThetaDriveDeadband),
                !m_driveJoystick.getRawButton(2), !m_driveJoystick.getRawButton(1)),
            m_robotDrive));

    m_chooser.setDefaultOption("PlaceConeDrive", "PlaceConeDrive");
    m_chooser.addOption("PlaceConeLevel", "PlaceConeLevel");
    m_chooser.addOption("PrototypeLeveling", "PrototypeLeveling");
    m_chooser.addOption("TrajectoryFollowingTest", "TrajectoryFollowingTest");
    m_chooser.addOption("PrototypeTrajectoryDriveOutLeveling", "PrototypeTrajectoryDriveOutLeveling");
    m_chooser.addOption("PrototypeDriveOutLevel", "PrototypeDriveOutLevel");
    SmartDashboard.putData("Auto choices", m_chooser);
    

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory trajectory0 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)),
        List.of(new Translation2d(1,0)),
        new Pose2d(3,0, new Rotation2d()),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    var xController = new PIDController(AutoConstants.kPXController, 0, 0);
    var yController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(
        trajectory0,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        xController,
        yController,
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
        trajectory1,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        xController,
        yController,
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
    
    

    
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory0.getInitialPose());


    
    autoMap.put("PlaceConeDrive", new SequentialCommandGroup(
        new WristToMiddlePeg(m_wrist),
        new ArmExtend(m_arm),
        new DoTimeOut(.8),
        new ClawOpen(m_claw),
        new ParallelCommandGroup(
            new AutoDriveFor(m_driveSubsystem, .5, 0, 0, false, 4),
            new WristToDrive(m_wrist)
        )
        //new InstantCommand(()-> setJoystickDriving())
    ));
        
    autoMap.put("PlaceConeDriveOutLevel", new SequentialCommandGroup(
        new WristToMiddlePeg(m_wrist),
        new ArmExtend(m_arm),
        new DoTimeOut(.8),
        new ClawOpen(m_claw),
        new ParallelCommandGroup(
            new AutoDriveFor(m_driveSubsystem, .7,0,0, false, 3.036),
            new WristToDrive(m_wrist)),
        new DoTimeOut(.2),
        new AutoDriveFor(m_driveSubsystem, -.5, 0,0, false, .6),
        new ChargeStationLevel(m_driveSubsystem, true),
        new AutoDriveFor(m_driveSubsystem, .1, 0, 0, false, .35)
        //new InstantCommand(()-> setJoystickDriving())
                
    ));
    
    
    autoMap.put("PlaceConeLevel", new SequentialCommandGroup(
        new WristToMiddlePeg(m_wrist),
        new ArmExtend(m_arm),
        new DoTimeOut(.8),
        new ClawOpen(m_claw),
        new SetBrake(m_driveSubsystem),
        new ParallelCommandGroup(
            new AutoDriveFor(m_driveSubsystem, .5, 0, 0, false, .7),
            new WristToDrive(m_wrist)
        ),
        new ChargeStationLevel(m_driveSubsystem, true),
        new AutoDriveFor(m_driveSubsystem, -.1, 0, 0, false, .35)

        //new InstantCommand(()-> setJoystickDriving())
    ));

    autoMap.put("PrototypeLeveling", new SequentialCommandGroup(
        new PrototypeChargeLeveling(m_driveSubsystem, AutoBalanceDirection.BACKWARD)
    ));

    autoMap.put("TrajectoryFollowingTest", new SequentialCommandGroup(
        swerveControllerCommand0,
        new InstantCommand(() -> m_driveSubsystem.drive(0,0,0,false,false))
    ));
    
    autoMap.put("PrototypeTrajectoryDriveOutLeveling", new SequentialCommandGroup(
        new WristToMiddlePeg(m_wrist),
        new ArmExtend(m_arm),
        new DoTimeOut(.8),
        new ClawOpen(m_claw),
        new SetBrake(m_driveSubsystem),
        swerveControllerCommand1,
        new PrototypeChargeLeveling(m_driveSubsystem, AutoBalanceDirection.FORWARD)
    ));

    autoMap.put("PrototypeDriveOutLevel", new SequentialCommandGroup(
        new WristToMiddlePeg(m_wrist),
        new ArmExtend(m_arm),
        new DoTimeOut(.8),
        new ClawOpen(m_claw),
        new ParallelCommandGroup(
            new AutoDriveFor(m_driveSubsystem, .4, 0, 0, false, 5),
            new WristToMiddleShelf(m_wrist)
        ),
        new PrototypeChargeLeveling(m_driveSubsystem, AutoBalanceDirection.FORWARD)
    ));

    

            
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
    new JoystickButton(m_driveJoystick, 8)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
        
    new JoystickButton(m_actuatorJoystick, Button.kLeftBumper.value).onTrue(m_clawClose);
    new JoystickButton(m_actuatorJoystick, Button.kRightBumper.value).onTrue(m_clawOpen);
    new JoystickButton(m_driveJoystick, 4).onTrue(m_armRetract);
    new JoystickButton(m_driveJoystick, 6).onTrue(m_armExtend);
    new JoystickButton(m_driveJoystick, 10).onTrue(m_setBrake);
    new JoystickButton(m_driveJoystick, 12).onTrue(m_setCoast);
    new JoystickButton(m_actuatorJoystick, Button.kA.value).onTrue(m_wristToBottom);
    new JoystickButton(m_actuatorJoystick, Button.kB.value).onTrue(m_wristToFeederStation);
    new JoystickButton(m_actuatorJoystick, Button.kX.value).onTrue(m_wristToMiddlePeg);
    new JoystickButton(m_actuatorJoystick, Button.kY.value).onTrue(m_wristToStart);
    new JoystickButton(m_actuatorJoystick, Button.kStart.value).onTrue(m_wristToTopPeg);
    new JoystickButton(m_actuatorJoystick, Button.kBack.value).onTrue(m_wristToTopShelf);
    new JoystickButton(m_actuatorJoystick, Button.kRightStick.value).onTrue(m_wristToMiddleShelf);
    new JoystickButton(m_actuatorJoystick, Button.kLeftStick.value).onTrue(m_wristToDrive);
    new JoystickButton(m_driveJoystick, 3).onTrue(m_solidPurple);
    new JoystickButton(m_driveJoystick, 5).onTrue(m_solidYellow);
    new JoystickButton(m_driveJoystick, 11).onTrue(m_flashy);
    new JoystickButton(m_driveJoystick, 8).onTrue(new InstantCommand(() -> m_orchestra.play()));
    new JoystickButton(m_driveJoystick, 7).onTrue(new InstantCommand(() -> m_orchestra.pause()));
    new JoystickButton(m_driveJoystick, 9).onTrue(new InstantCommand(() -> m_orchestra.stop()));
    
  }

  /* 
  private void setJoystickDriving(){
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driveJoystick.getY(), OIConstants.kXDriveDeadband),
                -MathUtil.applyDeadband(m_driveJoystick.getX(), OIConstants.kYDriveDeadband),
                -MathUtil.applyDeadband(m_driveJoystick.getZ(), OIConstants.kThetaDriveDeadband),
                !m_driveJoystick.getRawButton(2), !m_driveJoystick.getRawButton(1)),
            m_robotDrive));
  }
  */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    
    // Run path following command, then stop at the end.
    return autoMap.get(m_autoSelected);
  }
}
