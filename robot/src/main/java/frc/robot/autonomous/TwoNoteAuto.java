package frc.robot.autonomous;

import frc.robot.subsystems.*;
import frc.robot.util.*;

//Smart Dashboard library
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Libraries for Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TwoNoteAuto {
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
    public TwoNoteAuto(LemonDrive lemonDrive, LemonGrab lemonGrab, Util util, double startTime){
        //Setup utils
        this.util = util;
        this.lemonDrive = lemonDrive;
        this.startTime = startTime;
        this.lemonGrab = lemonGrab;

        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void run(){
        if(util.wait(startTime, 2)){
            //Lower arm and start spinning shoot wheels
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
        } else if (util.wait(startTime, 4)) {
            //Shoot
            lemonGrab.shoot();
        } else if (util.wait(startTime,6)) {
            //Drive, intake 
            lemonGrab.moveArmToPos(lemonGrab.kArmPosFloor);
            lemonDrive.gyroDrive(0.5,0);
            if (!lemonGrab.hasNote()){
                lemonGrab.intake();
            } else {
                lemonGrab.turnOff();
            }
        } else if (util.wait(startTime,10)) {
            //Drive back and start spinning shoot wheels
            lemonDrive.gyroDrive(-0.5, 0);
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
            if (!lemonGrab.hasNote()){
                lemonGrab.intake();
            }
        } else if (util.wait(startTime,14)) {
            //Shoot and stop
            lemonGrab.shoot();
            lemonDrive.gyroDrive(0,0);
        } else {
            //Stop everything
            lemonGrab.turnOff();
            lemonDrive.gyroDrive(0, 0);
        } 
    }
}
