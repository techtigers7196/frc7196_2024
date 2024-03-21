package frc.robot.subsystems;

//Libraries for motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

//Libraries for encoder
import edu.wpi.first.wpilibj.DutyCycleEncoder;

//Library for PID for the arm
import edu.wpi.first.math.controller.PIDController;

//Library for the color sensor for note detection
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

//Smart Dashboard library
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LemonGrab {

    //Setup our arm and encoder
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);

    //PID Variables for arm power
    private final double kP = 15;
    private final double kI = 0.5;
    private final double kD = 0;
    private final double feedForward = 0.01;

    //Max power for arm
    private final double maxArmPower = 0.7;

    //PID Controller for calculating arm power
    private final PIDController pid = new PIDController(kP, kI, kD);

    //Setpoint options
    public static double kArmPosFloor = 0.250; 
    public static double kArmPosSpeaker = 0.287;
    public static double kArmPosStart = 0.492;
    public static double kArmPosAmp = 0.5;
    public static double kArmPosExtra = 0.36;

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
     * Color sensor setup
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch colorMatcher = new ColorMatch();
    private final Color orangeTarget = new Color(0.56, 0.35, 0.08);
    private final Color redTarget = new Color(0.143, 0.427, 0.429);
    private final Color greenTarget = new Color(0.197, 0.561, 0.240);
    private final Color blueTarget = new Color(0.561, 0.232, 0.114);
    private final Color yellowTarget = new Color(0.361, 0.524, 0.113);

    //Contructor
    public LemonGrab() {
        //Set the top motor for the shooter to spin in reverse
        //shooterMotor.setInverted(true);
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.setIdleMode(IdleMode.kBrake);


        //Set the second arm motor to follow the first
        armMotorLeader.setInverted(true);
        armMotorLeader.setIdleMode(IdleMode.kBrake);
        armMotorFollower.setIdleMode(IdleMode.kBrake);
        armMotorFollower.follow(armMotorLeader, true);

        colorMatcher.addColorMatch(orangeTarget);
        colorMatcher.addColorMatch(redTarget);
        colorMatcher.addColorMatch(greenTarget);
        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(yellowTarget);
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
     * Function to check if the note is in the intake
     * We might have to do a color match instead if this doesn't work
     * https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java
     * @return boolean
     */
    public boolean hasNote() {
        int proximity = 0;
        proximity = colorSensor.getProximity();
        return proximity > 150;
    }

    /*
     * Push our Color Sensor Value to the SmartDashboard
     * This function will need to be called in robotPeriodic
     */
    public void pushColorSensorValue() {
        SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
        SmartDashboard.putBoolean("Has Note", this.hasNote());

    }

    public ColorMatchResult getColorMatch() {
        Color detectedColor = colorSensor.getColor();
        return colorMatcher.matchClosestColor(detectedColor);
    }

    /*
     * Push our Arm Encoder Value to the SmartDashboard
     * This function will need to be called in robotPeriodic
     */
    public void pushArmValue() {
        SmartDashboard.putNumber("Arm Position", this.getArmPosition());
    }

    //Translate the position to a value and move the arm to that position
    public void moveArmToPos(double armPosition) {
        double setpoint = 0;

        if(armPosition == kArmPosFloor) {
            setpoint = kArmPosFloor;
        } else if(armPosition == kArmPosSpeaker) {
            setpoint = kArmPosSpeaker;
        } else if(armPosition == kArmPosStart) {
            setpoint = kArmPosStart;
        } else if(armPosition == kArmPosAmp) {
            setpoint = kArmPosAmp;
        } else if(armPosition == kArmPosExtra){
            setpoint = kArmPosExtra;            
        }
         
        double pidValue = pid.calculate(this.getArmPosition(), setpoint);
        double power = feedForward + pidValue;
        this.moveArm(power);

        SmartDashboard.putNumber("PID", pidValue);
    }

    //Actually move the arm, while keeping the power under a max power
    public void moveArm(double power) {
        // Stop from making too much torque
        if (power > maxArmPower) {
            power = maxArmPower;
        } else if (power < -maxArmPower) {
            power = -maxArmPower;
        }

        //If arm gets outside of limits, reduce the power so we don't bend the frame
        if(this.getArmPosition() < (kArmPosFloor-.01) || this.getArmPosition() > (kArmPosAmp+.01)) {
            power = power / 10;
        } 
        
        armMotorLeader.set(power);
    }
    
    //get position of the arm 
    public double getArmPosition(){
        return armEncoder.getAbsolutePosition();
    }
}

