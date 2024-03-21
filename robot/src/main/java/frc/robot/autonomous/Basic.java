package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.*;

//Smart Dashboard library
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Basic {
    private Util util;
    private double startTime = 0;
    
    //Constructor
    public Basic(){
        //Setup utils
        util = new Util();

        startTime = util.timer.getFPGATimestamp();
        SmartDashboard.putNumber("Start time", startTime);
    }

    public void run(){
        
    }
}
