package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.json.JSONObject;


public class Drive extends SubsystemBase {
    private static TalonFX driveLeftFront;
    private static TalonFX driveLeftRear;
    private static TalonFX driveRightFront;
    private static TalonFX driveRightRear;

    private static TalonSRX steerLeftFront;    
    private static TalonSRX steerLeftRear;
    private static TalonSRX steerRightFront;
    private static TalonSRX steerRightRear;

    private static PIDController steerLeftFrontPID;
    private static PIDController steerLeftRearPID;
    private static PIDController steerRightFrontPID;
    private static PIDController steerRightRearPID;

    private static double inputLeftFront;        
    private static double inputLeftRear;
    private static double inputRightFront;
    private static double inputRightRear;

    public static final double WHEEL_BASE_LENGTH = 18;
    public static final double WHEEL_BASE_WIDTH = 24.5;
    public static final double ENCODER_COUNT_PER_ROTATION = 1024.0;

    // TODO: Find this value
    public static final double WHEEL_DIAMETER = 4.0;

    // TODO: increase MAX_SPEED
	public static final double MAX_SPEED = 0.3; // Max speed is 0 to 1
	public static final double STEER_DEGREES_PER_COUNT = 360.0 / ENCODER_COUNT_PER_ROTATION;
    // Jake - I literally have no idea what this math is or where these values are coming from but I aint gonna touch it
	public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67); 
	public static final double DEADZONE = 0.1;
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

    private static final double STEER_P = .2, STEER_I = 0.00, STEER_D = 0.002;
    private static final double RAMP_RATE = 0.5;
    public static final double OMEGA_SCALE = 1.0 / 30.0;
    private double originHeading = 0.0;
    private final double leftPow = 1.0;
	private final double rightPow = 1.0;

    public Drive(JSONObject config) {
        var current = config.getJSONObject("LeftFront");
        driveLeftFront = new TalonFX(current.getInt("SpeedMotor"));
        steerLeftFront = new TalonSRX(current.getInt("TurnMotor"));
        driveLeftFront.configFactoryDefault();
        driveLeftFront.setInverted(false);
        driveLeftFront.configOpenloopRamp(RAMP_RATE);

        current = config.getJSONObject("LeftRear");
        driveLeftRear = new TalonFX(current.getInt("SpeedMotor"));
        steerLeftRear = new TalonSRX(current.getInt("TurnMotor"));
        driveLeftRear.configFactoryDefault();
        driveLeftRear.setInverted(false);
        driveLeftRear.configOpenloopRamp(RAMP_RATE);

        current = config.getJSONObject("RightFront");
        driveRightFront = new TalonFX(current.getInt("SpeedMotor"));
        steerRightFront = new TalonSRX(current.getInt("TurnMotor"));
        driveRightFront.configFactoryDefault();
        driveRightFront.setInverted(false);
        driveRightFront.configOpenloopRamp(RAMP_RATE);

        current = config.getJSONObject("RightRear");
        driveRightRear = new TalonFX(current.getInt("SpeedMotor"));
        steerLeftFront = new TalonSRX(current.getInt("TurnMotor"));
        driveRightRear.configFactoryDefault();
        driveRightRear.setInverted(false);
        driveRightRear.configOpenloopRamp(RAMP_RATE);
        
        steerLeftFront.configFactoryDefault();
        steerLeftFront.setInverted(false);
        steerLeftFront.configOpenloopRamp(RAMP_RATE);

        steerLeftRear.configFactoryDefault();
        steerLeftRear.setInverted(false);
        steerLeftRear.configOpenloopRamp(RAMP_RATE);

        steerRightFront.configFactoryDefault();
        steerRightFront.setInverted(false);
        steerRightFront.configOpenloopRamp(RAMP_RATE);

        steerRightRear.configFactoryDefault();
        steerRightRear.setInverted(false);
        steerRightRear.configOpenloopRamp(RAMP_RATE);
        
        this.updateEncoderValues();

        steerLeftFrontPID = new PIDController(STEER_P, STEER_I, STEER_D);
        steerLeftFrontPID.enableContinuousInput(0, ENCODER_COUNT_PER_ROTATION);

        steerLeftRearPID = new PIDController(STEER_P, STEER_I, STEER_D);
        steerLeftRearPID.enableContinuousInput(0, ENCODER_COUNT_PER_ROTATION);

        steerRightFrontPID = new PIDController(STEER_P, STEER_I, STEER_D);
        steerRightFrontPID.enableContinuousInput(0, ENCODER_COUNT_PER_ROTATION);

        steerRightRearPID = new PIDController(STEER_P, STEER_I, STEER_D);
        steerRightRearPID.enableContinuousInput(0, ENCODER_COUNT_PER_ROTATION);
    }

    private void updateEncoderValues(){
        inputLeftFront = steerLeftFront.getSelectedSensorPosition(); 
        inputLeftRear =  steerLeftRear.getSelectedSensorPosition();
        inputRightFront = steerRightFront.getSelectedSensorPosition();
        inputRightRear = steerRightRear.getSelectedSensorPosition();    
    }
}
