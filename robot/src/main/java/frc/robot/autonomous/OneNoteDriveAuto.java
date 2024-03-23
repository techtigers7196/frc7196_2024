package frc.robot.autonomous;

import frc.robot.subsystems.*;
import frc.robot.util.*;

//Smart Dashboard library
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OneNoteDriveAuto {
    private Util util;
    private double startTime;
    private LemonDrive lemonDrive;
    private LemonGrab lemonGrab;
    
    //Constructor
    public OneNoteDriveAuto(LemonDrive lemonDrive, LemonGrab lemonGrab, Util util, double startTime){
        //Setup utils
        this.util = util;
        this.lemonDrive = lemonDrive;
        this.startTime = startTime;
        this.lemonGrab = lemonGrab;
    }

    public void run(){
        //Shoot, wait and then long drive
        if(util.wait(startTime, 3)){
            //Lower arm and start spinning shoot wheels
            lemonGrab.moveArmToPos(lemonGrab.kArmPosSpeaker);
            lemonGrab.spinFlyWheels(0.5);
          } else if (util.wait(startTime, 5)) {
            //Shoot
            lemonGrab.shoot();
          } else if (util.wait(startTime,8)) {
            //Wait
            lemonGrab.turnOff();
          } else if (util.wait(startTime,15)) {
            //Drive back
            lemonDrive.gyroDrive(-0.3, 0);
          } else {
            //Stop everything
            lemonDrive.gyroDrive(0, 0);
          }
    }
}
