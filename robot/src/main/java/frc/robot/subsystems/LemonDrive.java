package frc.robot.subsystems;

//Libraries for motor controllers
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class LemonDrive {
    //Max drivetrain speeds
    private double maxPower = 0.75;
    private double maxTurn = 0.6;

    //CAN ports for motor controllers
    private int leftDriveMotor1CANPort = 1;
    private int leftDriveMotor2CANPort = 2;
    private int rightDriveMotor1CANPort = 3;
    private int rightDriveMotor2CANPort = 4;

    //Motor controllers
    private CANSparkMax leftDriveMotorLeader = new CANSparkMax(leftDriveMotor1CANPort, MotorType.kBrushless);
    private CANSparkMax leftDriveMotorFollower = new CANSparkMax(leftDriveMotor2CANPort, MotorType.kBrushless);
    private CANSparkMax rightDriveMotorLeader = new CANSparkMax(rightDriveMotor1CANPort, MotorType.kBrushless);
    private CANSparkMax rightDriveMotorFollower = new CANSparkMax(rightDriveMotor2CANPort, MotorType.kBrushless);

    //Drive train
    private DifferentialDrive differentialDrive;

    public LemonDrive() {
        //Set motors to follow the leaders
        leftDriveMotorLeader.setInverted(true);
        leftDriveMotorFollower.follow(leftDriveMotorLeader);
        rightDriveMotorFollower.follow(rightDriveMotorLeader);

        //Setup drive
        differentialDrive = new DifferentialDrive(leftDriveMotorLeader, rightDriveMotorLeader);
    }

    public void drive(double power, double turn) {
        double adjustedPower = power * maxPower;
        double adjustedTurn = turn *maxTurn;

        differentialDrive.arcadeDrive(adjustedPower, adjustedTurn);
    }
    
}
