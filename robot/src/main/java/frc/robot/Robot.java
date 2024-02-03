// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share ixt under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Libraries for Xbox controller
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//Libraries for motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

//Libraries for Cameras
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final XboxController driveController = new XboxController(0);
  //private final XboxController operatorController = new XboxController(port:1);

  //Max drivetrain speeds
  private double maxForward = 0.75;
  private double maxTurn = 0.6;

  //arm 
  private CANSparkMax m_arm;
  private RelativeEncoder m_encoder;
  private final double kP = 0.016;
  private final double kI = 0.002;
  private final double kD = 0.0;
  private final PIDController pid = new PIDController(kP, kI, kD);
  private double setpoint = 0; 
  private double start = 1; 
  private double shooting = 41;


  //CAN ports for motor controllers
  private int leftDriveMotor1CANPort = 1;
  private int leftDriveMotor2CANPort = 2;
  private int rightDriveMotor1CANPort = 3;
  private int rightDriveMotor2CANPort = 4;
  private int shooterMotorLeaderCanPort = 5;
  private int shooterMotorFollowerCanPort = 6;
  private int armMotorLeaderCanPort = 7; 
  private int armMotorFollowerCanPort = 8;

  //Motor controllers
  private CANSparkMax leftDriveMotorLeader = new CANSparkMax(leftDriveMotor1CANPort, MotorType.kBrushless);
  private CANSparkMax leftDriveMotorFollower = new CANSparkMax(leftDriveMotor2CANPort, MotorType.kBrushless);
  private CANSparkMax rightDriveMotorLeader = new CANSparkMax(rightDriveMotor1CANPort, MotorType.kBrushless);
  private CANSparkMax rightDriveMotorFollower = new CANSparkMax(rightDriveMotor2CANPort, MotorType.kBrushless);
  private CANSparkMax shooterMotorLeader = new CANSparkMax(shooterMotorLeaderCanPort, MotorType.kBrushless);
  private CANSparkMax shooterMotorFollower = new CANSparkMax(shooterMotorFollowerCanPort, MotorType.kBrushless);
  private CANSparkMax armMotorLeader = new CANSparkMax (armMotorLeaderCanPort, MotorType.kBrushless);
  private CANSparkMax armMotorFollower = new CANSparkMax (armMotorFollowerCanPort, MotorType.kBrushless);


  //Drive train
  private DifferentialDrive differentialDrive;

  //Cameras
  private UsbCamera camera1;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Set motors to follow the leaders
    leftDriveMotorLeader.setInverted(true);
    leftDriveMotorFollower.follow(leftDriveMotorLeader);
    rightDriveMotorFollower.follow(rightDriveMotorLeader);

    shooterMotorLeader.setInverted(true);
    //shooterMotorFollower.follow(shooterMotorLeader, true);

    //Setup drive
    differentialDrive = new DifferentialDrive(leftDriveMotorLeader, rightDriveMotorLeader);

    //Setup front camera
    camera1 = CameraServer.startAutomaticCapture("Front Camera", 0);
    camera1.setResolution(320, 240);
    camera1.setFPS(15);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double forwardPower = driveController.getLeftY();
    double turnPower = driveController.getRightX();
    double shotTriggerAxis = driveController.getRightTriggerAxis();
    double shotSpeed = 0.2;

    differentialDrive.arcadeDrive(maxForward*forwardPower, maxTurn*turnPower);

    boolean aButtonPressed = driveController.getAButtonPressed();
    boolean yButtonPressed = driveController.getYButtonPressed();

    if(shotTriggerAxis > 0.5){
      shooterMotorLeader.set(shotSpeed);
      shooterMotorFollower.set(shotSpeed*2);
    } else {
      shooterMotorLeader.set(0);
      shooterMotorFollower.set(0);
    }
    
  }

    if(aButtonPressed){
    //set arm to starting position
    setpoint = start;
  } else if (yButtonPressed){
    //set arm to shooting position 
    setpoint = shooting; 

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
