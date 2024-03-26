package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

//Libraries for motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

//Library for drivetrain
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Library for gyro
import edu.wpi.first.wpilibj.AnalogGyro;

public class LemonDrive {
    //Max drivetrain speeds
    private double maxPower = 0.9;
    private double maxTurn = 0.7;

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

    //Gyro
    private AnalogGyro gyro = new AnalogGyro(0);

    //Constructor
    public LemonDrive() {
        //Set motors to follow the leaders
        leftDriveMotorLeader.setInverted(true);
        leftDriveMotorFollower.follow(leftDriveMotorLeader);
        rightDriveMotorFollower.follow(rightDriveMotorLeader);


        //Setup drive
        differentialDrive = new DifferentialDrive(leftDriveMotorLeader, rightDriveMotorLeader);

        gyro.calibrate();
    }

    //Drive
    public void drive(double power, double turn) {
        double adjustedPower = power * maxPower;
        double adjustedTurn = turn *maxTurn;

        differentialDrive.arcadeDrive(adjustedPower, adjustedTurn);
    }

    //Adjust our drive based on readings from our gyro
    //We use this during auto
    public void gyroDrive(double maxSpeed, double desiredAngle) {
        double Kp = 0.007;
        double currentAngle = gyro.getAngle();
        double error = (180 + desiredAngle - currentAngle) % 360 - 180;
        double steering = Kp * error;
        SmartDashboard.putNumber("Steering", steering);
        this.drive(maxSpeed, steering);
    }

    public double getGyroAngle() {
        return gyro.getAngle();
    }

    public void resetGyro() {
        gyro.reset();
    }
    
}
