package frc.robot.subsystems;

//Libraries for motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class LemonClimb {

    //can port for motors controllers
    private int climbMotorsCanPort = 9;

    //motor controllers
    private CANSparkMax climbMotors = new CANSparkMax(climbMotorsCanPort, MotorType.kBrushed);

    //Constructor
    public LemonClimb() {
        climbMotors.setInverted(true);
    }

    public void moveArm(double power) {
        climbMotors.set(power);
    }

}