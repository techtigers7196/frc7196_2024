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
    }

    public void run(){
        //Get limelight values
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");

        if(util.wait(startTime, 2)){
            //Lower arm and start spinning shoot wheels
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
        } else if (util.wait(startTime, 4)) {
            //Shoot
            lemonGrab.shoot();
        } else if (util.wait(startTime,5.5)) {
            //Drive, intake 
            lemonGrab.moveArmToPos(lemonGrab.kArmPosFloor);
            lemonDrive.gyroDrive(0.5,0);
            if (!lemonGrab.hasNote()){
                lemonGrab.intake();
            } else {
                lemonGrab.turnOff();
            }
        } else if (util.wait(startTime,9)) {
            //Shoot
            double armPosition = lemonGrab.calculateArmPosition(lemonGrab.getDistance(ty));
            lemonGrab.moveArmToPos(armPosition);

            double targetOffsetAngleHorizontal = tx.getDouble(0.0);
            double adjustmentAngle = Kp * targetOffsetAngleHorizontal;
            lemonDrive.drive(0, adjustmentAngle);
        } else if (util.wait(startTime,11)) {
            //Turn and intake
            lemonGrab.moveArmToPos(lemonGrab.kArmPosFloor);
            lemonDrive.gyroDrive(.375,-90);
            if(!lemonGrab.hasNote()) {
                lemonGrab.intake();
            } else {
                lemonGrab.turnOff();
            }
        } else if (util.wait(startTime,12)) {
            //Rough turn to angle
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
            lemonDrive.gyroDrive(0,-46);
        } else if (util.wait(startTime,15)) {
            //Shoot
            double armPosition = lemonGrab.calculateArmPosition(lemonGrab.getDistance(ty));
            lemonGrab.moveArmToPos(armPosition);

            double targetOffsetAngleHorizontal = tx.getDouble(0.0);
            double adjustmentAngle = Kp * targetOffsetAngleHorizontal;
            lemonDrive.drive(0, adjustmentAngle);
        } else {
            //Stop everything
            lemonGrab.turnOff();
            lemonDrive.gyroDrive(0, 0);
        } 
    }
}
