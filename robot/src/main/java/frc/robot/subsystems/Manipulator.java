package frc.robot.subsystems;

//Libraries for motor controllers
import com.revrobotics.CANSparkMax;

//Libraries for encoder
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

//Library for PID for the arm
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;

//Library for the color sensor for note detection
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;


public class Manipulator {

    //Setup our arm and encoder
    private RelativeEncoder armEncoder;

    //PID Variables for arm power
    private final double kP = 0.016;
    private final double kI = 0.002;
    private final double kD = 0.0;
    private final double feedForward = 0.01;

    //Max power for arm
    private final double maxArmPower = 0.5;

    //PID Controller for calculating arm power
    private final PIDController pid = new PIDController(kP, kI, kD);

    //Setpoint options
    public static double kArmPosFloor = 0; 
    public static double kArmPosFender = 40;
    public static double kArmPosStart = 30;
    public static double kArmPosAmp = 32;

    //CAN ports for motor controllers
    private int shooterMotorCanPort = 5;
    private int intakeMotorCanPort = 6;
    private int armMotorLeaderCanPort = 7; 
    private int armMotorFollowerCanPort = 8;

    //Motor controllers
    private CANSparkMax shooterMotor = new CANSparkMax(shooterMotorCanPort, MotorType.kBrushless);
    private CANSparkMax intakeMotor = new CANSparkMax(intakeMotorCanPort, MotorType.kBrushless);
    private CANSparkMax armMotorLeader = new CANSparkMax (armMotorLeaderCanPort, MotorType.kBrushless);
    private CANSparkMax armMotorFollower = new CANSparkMax (armMotorFollowerCanPort, MotorType.kBrushless);

    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    //Contructor
    public Manipulator() {
        //Set the top motor for the shooter to spin in reverse
        shooterMotor.setInverted(true);

        //Set the second arm motor to follow the first
        armMotorLeader.setIdleMode(IdleMode.kBrake);
        armMotorFollower.setIdleMode(IdleMode.kBrake);
        armMotorFollower.follow(armMotorLeader);
    }

    //Start the shooter spinning
    public void shoot(double shotSpeed) {
        shooterMotor.set(shotSpeed);
    }

    //Set the shooter to stop spinning
    public void stopShooting() {
        shooterMotor.set(0);
    }

    //Function to call our intake
    public void intake(double intakeSpeed) {
        intakeMotor.set(intakeSpeed);
    }
 
    /**
     * Function to check if the note is in the sensor
     * We might have to do a color match instead if this doesn't work
     * https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java
     * @return boolean
     */
    public boolean getNoteSensor() {
        int proximity = colorSensor.getProximity();
        return proximity > 0 && proximity < 100;
    }

    //Translate the position to a value and move the arm to that position
    public void moveArmToPos(double armPosition) {
        double setpoint = 0;

        if(armPosition == kArmPosFloor) {
            setpoint = kArmPosFloor;
        } else if(armPosition == kArmPosFender) {
            setpoint = kArmPosFender;
        } else if(armPosition == kArmPosStart) {
            setpoint = kArmPosStart;
        } else if(armPosition == kArmPosAmp) {
            setpoint = kArmPosAmp;
        }
         
        double pidValue = pid.calculate(armEncoder.getPosition(), setpoint);
        double power = feedForward + pidValue;
        this.moveArm(power);
    }

    //Actually move the arm, while keeping the power under a max power
    public void moveArm(double power) {
        // Stop from making too much torque
        if (power > maxArmPower) {
            power = maxArmPower;
        } else if (power < -maxArmPower) {
            power = -maxArmPower;
        }

        armMotorLeader.set(power);
    }
    
}
