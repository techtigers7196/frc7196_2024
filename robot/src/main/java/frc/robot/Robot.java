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

//Import subsytems
import frc.robot.subsystems.*;
import frc.robot.util.*;
import frc.robot.autonomous.*;

//Libraries for Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Auto config options
  private static final String kDriveAuto = "Drive Auto";
  private static final String kOneNoteDriveAuto = "One Note Drive Auto";
  private static final String kTwoNoteAuto = "Two Note Auto";
  private static final String kThreeNoteAuto = "Three Note Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Util util;
  private double startTime = 0;

  private DriveAuto driveAuto;
  private OneNoteDriveAuto oneNoteDriveAuto;
  private TwoNoteAuto twoNoteAuto;
  private ThreeNoteAuto threeNoteAuto;

  //Xbox Controller
  private final XboxController driveController = new XboxController(0);
  private final XboxController supportController = new XboxController(1);

  //Subsystems
  private LemonDrive lemonDrive;
  private LemonGrab lemonGrab;
  private LemonClimb lemonClimb;
  private int direction = 1; 

  //Arm position
  private double armPosition = lemonGrab.kArmPosStart;

  //Network tables
  private NetworkTable limelight;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private double Kp = 0.05;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Drive Auto", kDriveAuto);
    m_chooser.addOption(kOneNoteDriveAuto, kOneNoteDriveAuto);
    m_chooser.addOption(kTwoNoteAuto, kTwoNoteAuto);
    m_chooser.addOption(kThreeNoteAuto, kThreeNoteAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Setup drive subsytem
    lemonDrive = new LemonDrive();
    lemonGrab = new LemonGrab();
    lemonClimb = new LemonClimb();

    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    //Setup utils
    util = new Util();

    armPosition = lemonGrab.kArmPosFloor;
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
    //Push values from the arm to the dashboard
    lemonGrab.pushDashboardValues();

    //Get limelight values
    tx = limelight.getEntry("tx");
    SmartDashboard.putNumber("tx", tx.getDouble(0.0));
    ty = limelight.getEntry("ty");
    SmartDashboard.putNumber("ty", ty.getDouble(0.0));

    double distance = lemonGrab.getDistance(ty);
    SmartDashboard.putNumber("Distance", distance);

    double calculatedArmPosition = lemonGrab.calculateArmPosition(distance);
    SmartDashboard.putNumber("Arm Position: ", calculatedArmPosition);
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
    startTime = util.timer.getFPGATimestamp();
    SmartDashboard.putNumber("Start time", startTime);

    driveAuto = new DriveAuto(lemonDrive, util, startTime);
    oneNoteDriveAuto = new OneNoteDriveAuto(lemonDrive, lemonGrab, util, startTime);
    twoNoteAuto = new TwoNoteAuto(lemonDrive, lemonGrab, util, startTime);
    threeNoteAuto = new ThreeNoteAuto(lemonDrive, lemonGrab, util, startTime);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_autoSelected = m_chooser.getSelected();
    SmartDashboard.putNumber("Auto time", util.timer.getFPGATimestamp());
    SmartDashboard.putString("Auto selected", m_autoSelected);
    switch (m_autoSelected) {
      case kTwoNoteAuto:
        // Two note auto code goes here
        twoNoteAuto.run();
        break;
      case kOneNoteDriveAuto:
        //One note drive auto code goes here
        oneNoteDriveAuto.run();
        break;
      case kThreeNoteAuto:
        //Three note auto code here
        // IMPORTANT: CODE DOESNT DO ANYTHING YET
        threeNoteAuto.run();
        break;
      case kDriveAuto:
      default:
        // Put basic auto code here
        driveAuto.run();
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Drive code -------------------------------------------------------------------
    boolean backButtonPressed = driveController.getBackButtonPressed();

    if (backButtonPressed){
      direction = -direction;
    }   
  
    double forwardPower = direction * driveController.getLeftY();
    double turnPower = driveController.getRightX();
    lemonDrive.drive(forwardPower, turnPower);

    //Climbing code -------------------------------------------------------------------
    double climbPower = supportController.getLeftY();
    lemonClimb.moveArm(climbPower);
    SmartDashboard.putNumber("Climb power", climbPower);

    //Shooting and intake code -------------------------------------------------------------------
    boolean leftBumperPressed = driveController.getLeftBumper();
    boolean leftBumperReleased = driveController.getLeftBumperReleased();
    double leftTrigger = driveController.getLeftTriggerAxis();
    double rightTrigger = driveController.getRightTriggerAxis();
    boolean rightBumper = driveController.getRightBumper();
    boolean rightBumper2 = supportController.getRightBumper();
    boolean rightBumper2Released = supportController.getRightBumperReleased();
    boolean leftBumper2 = supportController.getLeftBumper();
    boolean leftBumper2Released = supportController.getLeftBumperReleased();
    

    if (leftBumperPressed && !lemonGrab.hasNote()){
      /*
       * If we are pressing the .intake button and there's no note in the intake
       * Move the arm to the floor position and start intaking
       */
      armPosition = lemonGrab.kArmPosFloor;
      lemonGrab.intake();  
    } else if (leftTrigger > 0 && lemonGrab.hasNote()) {
      //If we are not intaking, and we press the left trigger raise arm to amp position
      armPosition = lemonGrab.kArmPosAmp;
      if (lemonGrab.getArmPosition() >= lemonGrab.kArmPosAmp) {
        //If we're pressing the left trigger and in position, then shoot
        lemonGrab.spinFlyWheels(0.5);
        lemonGrab.spinFeederWheels(0.5);
      }
    } else if (rightTrigger > 0) {
      //If we are not intaking or scoring on the amp and press the right trigger then start shooting process
      if(rightTrigger < 0.5) {
        armPosition = lemonGrab.calculateArmPosition(lemonGrab.getDistance(ty));

        double targetOffsetAngleHorizontal = tx.getDouble(0.0);
        double adjustmentAngle = Kp * targetOffsetAngleHorizontal;
        lemonDrive.drive(forwardPower, adjustmentAngle);
      } else {
        lemonGrab.shoot();
      }
    } else {
      //If we aren't pressing any buttons turn the motors off
      lemonGrab.turnOff();
    }

    if(leftBumperPressed && lemonGrab.hasNote()){
      // if note is in the intake set the controller to vibrate and turn off the intake
      driveController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
      supportController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }else {
      //turn off rumble
      driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
      supportController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
    

    if (leftBumperReleased) {
      //No longer intaking; raise intake to avoid damage
      //armPosition = lemonGrab.kArmPosSpeaker;
    }

    if (rightBumper2){
      //If the support driver presses the right bumper spit out the note
      lemonGrab.spinFeederWheels(-1);
    } else if(rightBumper2Released){
      //When the support driver lets go of the right bumper set the intake back to 0
      lemonGrab.spinFeederWheels(0);
    } else if(leftBumper2){
      //when support driver presses the left bumper push back the note 
      lemonGrab.spinFlyWheels(-0.2);
    } else if (leftBumper2Released){
      //when the support driver lets go of the left bumper set the intake to 0
      lemonGrab.spinFlyWheels(0);
    }

    //Arm controls for the Primary Driver Controller -------------------------------------------------------------------
    boolean aButtonPressed = driveController.getAButtonPressed();
    boolean yButtonPressed = driveController.getYButtonPressed();
    boolean xButtonPressed = driveController.getXButtonPressed();
    boolean bButtonPressed = driveController.getBButtonPressed();

    //Arm controls for the Support driver controller -------------------------------------------------------------------
    boolean a2ButtonPressed = supportController.getAButtonPressed();
    boolean y2ButtonPressed = supportController.getYButtonPressed();
    boolean x2ButtonPressed = supportController.getXButtonPressed();
    boolean b2ButtonPressed = supportController.getBButtonPressed();
    boolean backButton2Pressed = supportController.getBackButtonPressed();

   if(aButtonPressed){
      //set arm to starting position
      armPosition = lemonGrab.kArmPosFloor;
    } else if (bButtonPressed){
      //set arm to shooting position 
      armPosition = lemonGrab.kArmPosSpeaker; 
    }else if (yButtonPressed){
      //set arm to idk what position it is 
      armPosition= lemonGrab.kArmPosStart;
    }else if (xButtonPressed){
      //set arm to amp position
      armPosition= lemonGrab.kArmPosAmp;
    }

    if(a2ButtonPressed) {
      armPosition = lemonGrab.kArmPosFloor2;
    }  else if ( b2ButtonPressed){
      //set arm to shooting position 
      armPosition = lemonGrab.kArmPosSpeaker2; 
    }else if (y2ButtonPressed){
      //set arm to idk what position it is 
      armPosition= lemonGrab.kArmPosStart2;
    }else if (x2ButtonPressed){
      //set arm to amp position
      armPosition= lemonGrab.kArmPosAmp2;
    }else if(backButton2Pressed){
      armPosition = lemonGrab.kArmPosExtra;
    }

    SmartDashboard.putNumber("Goal position", armPosition);

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
