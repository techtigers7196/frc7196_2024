package frc.robot.subsystems;

//Libraries for motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

//Libraries for encoder
import edu.wpi.first.wpilibj.DutyCycleEncoder;

//Library for PID for the arm
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
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

    //Setup our shoot encoder and variables
    private RelativeEncoder shootEncoder;
    private double shotPower = .9;
    private double shotSpeed = 4200;
    private double feedPower = .7;

    //Intake variables
    private double intakePower = 0.35;
    private double intakePrevent = -0.1;

    //PID Variables for arm power
    private final double kP = 15;
    private final double kI = 0.5;
    private final double kD = 0;
    private final double feedForward = 0.01;

    //Max power for arm
    private final double maxArmPower = 0.7;

    //PID Controller for calculating arm power
    private final PIDController pid = new PIDController(kP, kI, kD);

    // Setpoint options
    public static double kArmPosFloor = 0.247; 
    public static double kArmPosSpeaker = 0.287;
    public static double kArmPosStart = 0.492;
    public static double kArmPosAmp = 0.5;
    // public static double kArmPosExtra = 0.36;

    public static double kArmPosFloor2 = 0.32; 
    public static double kArmPosSpeaker2 = 0.342;
    public static double kArmPosStart2 = 0.355;
    public static double kArmPosAmp2 = 0.362 ;
    public static double kArmPosExtra = 0.359;

    //CAN ports for motor controllers
    private int flywheelMotorCanPort = 5;
    private int feederMotorCanPort = 6;
    private int armMotorLeaderCanPort = 7; 
    private int armMotorFollowerCanPort = 8;

    //Motor controllers
    private CANSparkMax flywheelMotor = new CANSparkMax(flywheelMotorCanPort, MotorType.kBrushless);
    private CANSparkMax feederMotor = new CANSparkMax(feederMotorCanPort, MotorType.kBrushless);
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
        //flywheelMotor.setInverted(true);
        feederMotor.setInverted(true);
        feederMotor.setIdleMode(IdleMode.kCoast);
        flywheelMotor.setIdleMode(IdleMode.kBrake);

        shootEncoder = flywheelMotor.getEncoder();


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

    /*
     * Start the shooter wheels spinning
     */
    public void spinFlyWheels(double speed) {
        flywheelMotor.set(speed);
    }

    /*
     * Function to call our intake wheels
     */
    public void spinFeederWheels(double speed) {
        feederMotor.set(speed);
    }

    /*
     * Function to intake a note
     */
    public void intake() {
        this.spinFeederWheels(intakePower);
        this.spinFlyWheels(intakePrevent);
    }

    /*
     * Function to shoot on the amp
     */
    public void shootAmp() {
        
    }

    /*
     * Function to shoot a note
     */
    public void shoot() {
        this.spinFlyWheels(shotPower);

        if(shootEncoder.getVelocity() >= shotSpeed) {
            this.spinFeederWheels(feedPower);
        }
    }

    /*
     * This function uses the limelight ty value to calculate the distance from the speaker
     * 
     * Returns a double distance in inches
     */
    public double getDistance(NetworkTableEntry ty) {
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 28; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 10.5; 

        // distance from the target to the floor
        double goalHeightInches = 49.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    /*
    * This function takes a distance and returns an arm position to score on the speaker
    * Works best within 12 feet of the speaker
    */
    public double calculateArmPosition(double distance) {
        double armPosition = 0.209 + 0.00231*distance - 0.0000131*Math.pow(distance, 2) + 0.0000000259*Math.pow(distance, 3);
        return armPosition;
    }

    /*
     * Function to turn off all of the motors
     */
    public void turnOff() {
        this.spinFeederWheels(0);
        this.spinFlyWheels(0);
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
        return proximity > 120;
    }

    /*
     * Push values to the SmartDashboard
     * This function will need to be called in robotPeriodic
     */
    public void pushDashboardValues() {
        SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
        SmartDashboard.putBoolean("Has Note", this.hasNote());

        SmartDashboard.putNumber("Velocity: ", shootEncoder.getVelocity());

        SmartDashboard.putNumber("Arm Position", this.getArmPosition());
    }

    public ColorMatchResult getColorMatch() {
        Color detectedColor = colorSensor.getColor();
        return colorMatcher.matchClosestColor(detectedColor);
    }

    //Translate the position to a value and move the arm to that position
    public void moveArmToPos(double armPosition) {
        double pidValue = pid.calculate(this.getArmPosition(), armPosition);
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

