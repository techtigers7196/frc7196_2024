package frc.robot.autonomous;

import frc.robot.subsystems.*;
import frc.robot.util.*;

//Smart Dashboard library
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Libraries for Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ThreeNoteAuto {
    private Util util;
    private double startTime;
    private LemonDrive lemonDrive;
    private LemonGrab lemonGrab;

    //Network tables
    private NetworkTable limelight;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private double Kp = 0.05;
    
    //Constructor
    public ThreeNoteAuto(LemonDrive lemonDrive, LemonGrab lemonGrab, Util util, double startTime){
        //Setup utils
        this.util = util;
        this.lemonDrive = lemonDrive;
        this.startTime = startTime;
        this.lemonGrab = lemonGrab;

        this.limelight = NetworkTableInstance.getDefault().getTable("limelight");

        this.lemonDrive.resetGyro();
    }

    public void run(){
        //Get limelight values
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");

        if(util.wait(startTime, 2)){
            //Lower arm and start spinning shoot wheels
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
        } else if (util.wait(startTime, 4)) {
            //Shoot and drive slowly into the speaker
            lemonGrab.shoot();
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
            lemonDrive.gyroDrive(-0.1,0, false);
        } else if (util.wait(startTime,6)) {
            //Drive, intake 
            lemonGrab.moveArmToPos(lemonGrab.kArmPosFloor);

            //If the robot is less than 123 inches drive, then stop
            if(lemonGrab.getDistance(ty) <= 120) {
                lemonDrive.gyroDrive(0.5,0,false);
            } else {
                lemonDrive.gyroDrive(0, 0, false);
            }
            
            if (!lemonGrab.hasNote()){
                lemonGrab.intake();
            } else {
                lemonGrab.turnOff();
            }
        } else if (util.wait(startTime,8)) {
            //Shoot
            double armPosition = lemonGrab.calculateArmPosition(lemonGrab.getDistance(ty));
            lemonGrab.moveArmToPos(armPosition);

            double targetOffsetAngleHorizontal = tx.getDouble(0.0);
            double adjustmentAngle = Kp * targetOffsetAngleHorizontal;
            lemonDrive.drive(0, adjustmentAngle);

            lemonGrab.shoot();
        } else if (util.wait(startTime,10)) {
            //Turn and intake
            lemonGrab.moveArmToPos(lemonGrab.kArmPosFloor);
            lemonDrive.gyroDrive(0, 90, true);
              
        } else if (util.wait(startTime, 11.7))
        {
            lemonDrive.gyroDrive(0.5, 90, true);
            if(!lemonGrab.hasNote()) {
                lemonGrab.intake();
            } else {
                lemonGrab.turnOff();
            }
        }
        else if (util.wait(startTime,13)) {
             //Rough turn to angle
             lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
             lemonDrive.gyroDrive(0,15,true);
        }
        else if (util.wait(startTime,15)) {
        //     //Shoot
            double armPosition = lemonGrab.calculateArmPosition(lemonGrab.getDistance(ty));
            lemonGrab.moveArmToPos(armPosition);

            double targetOffsetAngleHorizontal = tx.getDouble(0.0) +4;
            double adjustmentAngle = Kp * targetOffsetAngleHorizontal;
            lemonDrive.drive(0, adjustmentAngle);

            lemonGrab.shoot();
        } 
        else {
            //Stop everything
            lemonGrab.turnOff();
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
            lemonDrive.gyroDrive(0, 85, false);
        } 
        // } else if (util.wait(startTime,12)) {
        //     //Rough turn to angle
        //     lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
        //     lemonDrive.gyroDrive(0,-46);
        // } else if (util.wait(startTime,15)) {
        //     //Shoot
        //     double armPosition = lemonGrab.calculateArmPosition(lemonGrab.getDistance(ty));
        //     lemonGrab.moveArmToPos(armPosition);

        //     double targetOffsetAngleHorizontal = tx.getDouble(0.0);
        //     double adjustmentAngle = Kp * targetOffsetAngleHorizontal;
        //     lemonDrive.drive(0, adjustmentAngle);
        // } 
        
    }
}
