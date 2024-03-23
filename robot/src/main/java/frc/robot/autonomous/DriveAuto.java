package frc.robot.autonomous;

import frc.robot.subsystems.LemonDrive;
import frc.robot.util.*;

//Smart Dashboard library
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAuto {
    private Util util;
    private double startTime;
    private LemonDrive lemonDrive;
    
    //Constructor
    public DriveAuto(LemonDrive lemonDrive, Util util, double startTime){
        //Setup utils
        this.util = util;
        this.lemonDrive = lemonDrive;
        this.startTime = startTime;
    }

    public void run(){
        //Drive forward for three seconds
        if(util.wait(startTime, 3 )){
            lemonDrive.gyroDrive(0.5, 0);
        } else {
            lemonDrive.gyroDrive(0, 0);
        }
    }
}
