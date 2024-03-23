package frc.robot.autonomous;

import frc.robot.subsystems.*;
import frc.robot.util.*;

//Smart Dashboard library
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TwoNoteAuto {
    private Util util;
    private double startTime;
    private LemonDrive lemonDrive;
    private LemonGrab lemonGrab;
    
    //Constructor
    public TwoNoteAuto(LemonDrive lemonDrive, LemonGrab lemonGrab, Util util, double startTime){
        //Setup utils
        this.util = util;
        this.lemonDrive = lemonDrive;
        this.startTime = startTime;
        this.lemonGrab = lemonGrab;
    }

    public void run(){
        if(util.wait(startTime, 3)){
            //Lower arm and start spinning shoot wheels
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
            lemonGrab.spinFlyWheels(0.5);
        } else if (util.wait(startTime, 5)) {
            //Shoot
            lemonGrab.spinFlyWheels(0.5);
            lemonGrab.spinFeederWheels(.5);
        } else if (util.wait(startTime,7.5)) {
            //Drive, intake
            lemonGrab.spinFlyWheels(0); 
            lemonGrab.moveArmToPos(lemonGrab.kArmPosFloor);
            lemonDrive.gyroDrive(0.5,0);
            if (!lemonGrab.hasNote()){
                lemonGrab.spinFeederWheels(.44);
                lemonGrab.spinFlyWheels(-0.05);
            } else {
                lemonGrab.spinFeederWheels(0);
                lemonGrab.spinFlyWheels(0);
            }
        } else if (util.wait(startTime,11.5)) {
            //Drive back and start spinning shoot wheels
            lemonDrive.gyroDrive(-0.5, 0);
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
            if (!lemonGrab.hasNote()){
                lemonGrab.spinFeederWheels(.44);
                lemonGrab.spinFlyWheels(-0.05);
            } else {
                lemonGrab.spinFeederWheels(0);
                lemonGrab.spinFlyWheels(0.5);
            }
        } else if (util.wait(startTime,14)) {
            //Shoot and stop
            lemonGrab.spinFlyWheels(0.5);
            lemonGrab.spinFeederWheels(0.5);
            lemonDrive.gyroDrive(0,0);
        } else {
            //Stop everything
            lemonGrab.spinFeederWheels(0);
            lemonGrab.spinFlyWheels(0);
            lemonDrive.gyroDrive(0, 0);
        } 
    }
}
