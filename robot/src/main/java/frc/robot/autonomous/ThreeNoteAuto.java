package frc.robot.autonomous;

import frc.robot.subsystems.*;
import frc.robot.util.*;

//Smart Dashboard library
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ThreeNoteAuto {
    private Util util;
    private double startTime;
    private LemonDrive lemonDrive;
    private LemonGrab lemonGrab;
    
    //Constructor
    public ThreeNoteAuto(LemonDrive lemonDrive, LemonGrab lemonGrab, Util util, double startTime){
        //Setup utils
        this.util = util;
        this.lemonDrive = lemonDrive;
        this.startTime = startTime;
        this.lemonGrab = lemonGrab;
    }

    public void run(){
        
    }
}
