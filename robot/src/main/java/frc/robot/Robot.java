// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share ixt under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Libraries for Xbox controller
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

//Libraries for Cameras
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

//Import subsytems
import frc.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Auto config options
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Xbox Controller
  private final XboxController driveController = new XboxController(0);

  //Cameras
  private UsbCamera camera1;

  //Subsystems
  private LemonDrive lemonDrive;
  private LemonGrab lemonGrab;

  double armPosition;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Setup drive subsytem
    lemonDrive = new LemonDrive();
    lemonGrab = new LemonGrab();

    armPosition = lemonGrab.kArmPosFloor;


    //Setup front camera
    // camera1 = CameraServer.startAutomaticCapture("Front Camera", 0);
    // camera1.setResolution(320, 240);
    // camera1.setFPS(15);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    lemonGrab.pushArmValue();
    lemonGrab.pushColorSensorValue();
  }

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

    //Drive
    double forwardPower = driveController.getLeftY();
    double turnPower = driveController.getRightX();
    lemonDrive.drive(forwardPower, turnPower);

    // //intake
    // if(driveController.getRightBumperPressed() && !manipulator.hasNote()){
    //   //if pressing intake button and the  note is not in the  intake
    //   manipulator.intake(0.375); 
      
    //   //if we are not shooting 
    //   if(driveController.getRightTriggerAxis() <0.5 ){
    //      armPosition = manipulator.kArmPosFloor;
    //   }
    // } else if (driveController.getLeftBumperPressed()){
    //   //if we press the left bumper load the note 
    //   manipulator.intake(-1);
    //   manipulator.shoot(-0.25);
    // } else {
    //   //if no bumpers are pressed turn off the intake and shooter
    //   manipulator.intake(0);
    //   manipulator.stopShooting();
    // }

    //  //shooter  
    //  if (driveController.getRightTriggerAxis()>0.1){
    //   //if statring to press the trigger it will start shooting
    //   if(manipulator.getArmEncoder()<manipulator.kArmPosStart){
    //     //dont understand quokkas code
    //     manipulator.shoot(0.25);
    //   }
    // }
        
    // if(driveController.getRightBumperPressed() && manipulator.hasNote()){
    //   // if note is in the intake set the controller to vibrate
    //   driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    // }else {
    //   //turn off rumble
    //   driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    // }
    

    // if (driveController.getRightBumperReleased()) {
    //   //No longer intaking; raise intake to avoid damage
    //   armPosition = manipulator.kArmPosFender;
    // }

    //Arm
    boolean aButtonPressed = driveController.getAButtonPressed();
    boolean yButtonPressed = driveController.getYButtonPressed();
    boolean xButtonPressed = driveController.getXButtonPressed();
    boolean bButtonPressed = driveController.getBButtonPressed();


   if(aButtonPressed){
      //set arm to starting position
      armPosition = lemonGrab.kArmPosFloor;
    } else if (bButtonPressed){
      //set arm to shooting position 
      armPosition = lemonGrab.kArmPosFender; 
    }else if (yButtonPressed){
      //set arm to idk what position it is 
      armPosition= lemonGrab.kArmPosStart;
    }else if (xButtonPressed){
      //set arm to amp position
      armPosition= lemonGrab.kArmPosAmp;
    }

    lemonGrab.moveArmToPos(armPosition);

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
