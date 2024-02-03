package frc.robot.subsystems;

//Libraries for motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Manipulator {

    //Setup our arm and encoder
    private RelativeEncoder armEncoder;

    //PID Variable
    private final double kP = 0.016;
    private final double kI = 0.002;
    private final double kD = 0.0;
    private final double feedForward = 0.01;

    //PID Controller
    private final PIDController pid = new PIDController(kP, kI, kD);

    //Setpoints options
    public static String kArmPosStart = "Start"; 
    public static String kArmPosShooting = "Shooting";

    //Setpoint values
    private final double armPosStartValue = 0;
    private final double armPosShootingValue = 40;

    //CAN ports for motor controllers
    private int shooterMotorTopCanPort = 5;
    private int shooterMotorBottomCanPort = 6;
    private int armMotorLeaderCanPort = 7; 
    private int armMotorFollowerCanPort = 8;

    //Motor controllers
    private CANSparkMax shooterMotorTop = new CANSparkMax(shooterMotorTopCanPort, MotorType.kBrushless);
    private CANSparkMax shooterMotorBottom = new CANSparkMax(shooterMotorBottomCanPort, MotorType.kBrushless);
    private CANSparkMax armMotorLeader = new CANSparkMax (armMotorLeaderCanPort, MotorType.kBrushless);
    private CANSparkMax armMotorFollower = new CANSparkMax (armMotorFollowerCanPort, MotorType.kBrushless);

    public Manipulator() {
        //Set the top motor for the shooter to spin in reverse
        shooterMotorTop.setInverted(true);

        //Set the second arm motor to follow the first
        armMotorFollower.follow(armMotorLeader);
    }

    public void shoot(double shotSpeed) {
        shooterMotorTop.set(shotSpeed);
        shooterMotorBottom.set(shotSpeed*2);
    }

    public void stopShooting() {
        shooterMotorTop.set(0);
        shooterMotorBottom.set(0);
    }

    public void moveArmToPos(String armPosition) {
        double setpoint = 0;

        if(armPosition == kArmPosStart) {
            setpoint = armPosStartValue;
        } else if(armPosition == kArmPosShooting) {
            setpoint = armPosShootingValue;
        }
         
        double pidValue = pid.calculate(armEncoder.getPosition(), setpoint);
        armMotorLeader.set(feedForward + pidValue);
    }
    
}
