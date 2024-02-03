package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class Util {
    Timer timer = new Timer();

    public boolean wait(double stepStart, double duration) {
        return timer.getFPGATimestamp() < (stepStart + duration);
    }
}
