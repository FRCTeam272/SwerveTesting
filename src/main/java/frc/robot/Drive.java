package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.Collections;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
		System.out.println("Making DriveTrain");
        var current = config.getJSONObject("LeftFront");
        
		driveLeftFront = new TalonFX(current.getInt("SpeedMotor"));
        steerLeftFront = new TalonSRX(current.getInt("TurnMotor"));
        
		driveLeftFront.configFactoryDefault();
        driveLeftFront.setInverted(false);
        driveLeftFront.configOpenloopRamp(RAMP_RATE);

		steerLeftFront.configFactoryDefault();
        steerLeftFront.setInverted(false);
        steerLeftFront.configOpenloopRamp(RAMP_RATE);


        current = config.getJSONObject("LeftRear");
        driveLeftRear = new TalonFX(current.getInt("SpeedMotor"));
        steerLeftRear = new TalonSRX(current.getInt("TurnMotor"));

        driveLeftRear.configFactoryDefault();
        driveLeftRear.setInverted(false);
        driveLeftRear.configOpenloopRamp(RAMP_RATE);

		steerLeftRear.configFactoryDefault();
        steerLeftRear.setInverted(false);
        steerLeftRear.configOpenloopRamp(RAMP_RATE);


        current = config.getJSONObject("RightFront");
        driveRightFront = new TalonFX(current.getInt("SpeedMotor"));
        steerRightFront = new TalonSRX(current.getInt("TurnMotor"));
        
		driveRightFront.configFactoryDefault();
        driveRightFront.setInverted(false);
        driveRightFront.configOpenloopRamp(RAMP_RATE);

		steerRightFront.configFactoryDefault();
        steerRightFront.setInverted(false);
        steerRightFront.configOpenloopRamp(RAMP_RATE);

        current = config.getJSONObject("RightRear");
        driveRightRear = new TalonFX(current.getInt("SpeedMotor"));
        steerRightRear = new TalonSRX(current.getInt("TurnMotor"));

        driveRightRear.configFactoryDefault();
        driveRightRear.setInverted(false);
        driveRightRear.configOpenloopRamp(RAMP_RATE);
        
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

    public void drive(double leftY, double leftX, double rightX, boolean fieldCentric)
    {
        SmartDashboard.putNumber("LeftY: ", leftY);
		SmartDashboard.putNumber("leftX", leftX);
		SmartDashboard.putNumber("rightX", rightX);
		SmartDashboard.putBoolean("fieldCentric", fieldCentric);

        double strafe = Math.pow(Math.abs(leftX), leftPow)
				* Math.signum(leftX);
		double forward = Math.pow(Math.abs(leftY), leftPow)
				* -Math.signum(leftY);
		double omega = Math.pow(Math.abs(rightX), rightPow)
				* Math.signum(rightX) * OMEGA_SCALE;

        if (Math.abs(strafe) < Math.pow(DEADZONE, leftPow))
			strafe = 0.0;
		if (Math.abs(forward) < Math.pow(DEADZONE, leftPow))
			forward = 0.0;
		if (Math.abs(omega) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE)
			omega = 0.0;

        SmartDashboard.putNumber("Strafe", strafe);
        SmartDashboard.putNumber("Forward", forward);
        SmartDashboard.putNumber("omega", omega);

        // If all of the joysticks are in the deadzone, don't update the motors
		// This makes side-to-side strafing much smoother
		if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
			this.setDriveLeftFront(0.0);
			this.setDriveLeftRear(0.0);
			this.setDriveRightFront(0.0);
			this.setDriveRightRear(0.0);
			this.setSteerLeftFront(0.0);
			this.setSteerLeftRear(0.0);
			this.setSteerRightFront(0.0);
			this.setSteerRightRear(0.0);
			return;
		}

        if (!fieldCentric) {
			// When the Left Joystick trigger is not pressed, The robot is in Field Centric
			// Mode.
			// The calculations correct the forward and strafe values for field centric
			// attitude.

			// Rotate the velocity vector from the joystick by the difference between our
			// current orientation and the current origin heading
			final double originCorrection = Math.toRadians(originHeading - Navx.getInstance().navX.getFusedHeading());
			final double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
			strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
			forward = temp;
		}

        this.swerveDrive(strafe, forward, omega, true);
    }

    public void swerveDrive(double strafe, double forward, double omega, boolean fieldCentric) {
		double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
		double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);

		// Compute the constants used later for calculating speeds and angles
		double A = strafe - omegaL2;
		double B = strafe + omegaL2;
		double C = forward - omegaW2;
		double D = forward + omegaW2;

		// Compute the drive motor speeds
		double speedLF = speed(B, D);
		double speedLR = speed(A, D);
		double speedRF = speed(B, C);
		double speedRR = speed(A, C);

		// ... and angles for the steering motors
		// When drives are calibrated for zero position on encoders they are at 90
		// degrees
		// to the front of the robot. Subtract and add 90 degrees to steering
		// calculation to offset
		// for initial position/calibration of drives.

		double angleLF = angle(B, D) - 90;
		double angleLR = angle(A, D) + 90;
		double angleRF = angle(B, C) - 90;
		double angleRR = angle(A, C) + 90;
		// Compute the maximum speed so that we can scale all the speeds to the range
		// [0, 1]
		double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));


        // The method 
        // setSwerveModule(String, PIDController, TalonFX, TalonSRX, double, double) in the type Drive is not applicable for the arguments 
        // (String, double, PIDController, TalonFX, TalonSRX, double, double)
		// Set each swerve module, scaling the drive speeds by the maximum speed
		setSwerveModule("LF", steerLeftFrontPID, driveLeftFront, steerLeftFront, angleLF, speedLF / maxSpeed);
		setSwerveModule("LR", steerLeftRearPID, driveLeftRear, steerLeftRear, angleLR, speedLR / maxSpeed);
		setSwerveModule("RF", steerRightFrontPID, driveRightFront, steerRightFront, angleRF, speedRF / maxSpeed);
		setSwerveModule("RR", steerRightRearPID ,driveRightRear, steerRightRear, angleRR, speedRR / maxSpeed);
	}


    private double speed(double val1, double val2) {
		return Math.hypot(val1, val2);
	}

	private double angle(double val1, double val2) {
		return Math.toDegrees(Math.atan2(val1, val2));
	}

    public void setDriveLeftFront(double speed) {
		driveLeftFront.set(ControlMode.PercentOutput, speed);
	}

	public void setDriveLeftRear(double speed) {
		driveLeftRear.set(ControlMode.PercentOutput, speed);
	}

	public void setDriveRightFront(double speed) {
        driveRightFront.set(ControlMode.PercentOutput, speed);
	}
	public void setDriveRightRear(double speed) {
		driveRightRear.set(ControlMode.PercentOutput, speed);
	}

	public void setSteerLeftFront(double speed) {
		steerLeftFront.set(ControlMode.Position, speed);
	}

	public void setSteerLeftRear(double speed) {
		steerLeftRear.set(ControlMode.Position, speed);
	}

	public void setSteerRightFront(double speed) {
		steerRightFront.set(ControlMode.Position, speed);
	}

	public void setSteerRightRear(double speed) {
		steerRightRear.set(ControlMode.Position, speed);
	}

    private void setSwerveModule(String name, PIDController pidController, TalonFX drive, TalonSRX steer, double angle, double speed) {
		double currentPosition = steer.getSelectedSensorPosition();
		
		double currentAngle = (currentPosition * 360.0 / ENCODER_COUNT_PER_ROTATION) % 360.0;
		// The angle from the encoder is in the range [0, 360], but the swerve
		// computations
		// return angles in the range [-180, 180], so transform the encoder angle to
		// this range
		if (currentAngle > 180.0) {
			currentAngle -= 360.0;
		}
		// TODO: Properly invert the steering motors so this isn't necessary
		// This is because the steering encoders are inverted
		double targetAngle = -angle;
		double deltaDegrees = targetAngle - currentAngle;
		// If we need to turn more than 180 degrees, it's faster to turn in the opposite
		// direction
		if (Math.abs(deltaDegrees) > 180.0) {
			deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
		}
		// If we need to turn more than 90 degrees, we can reverse the wheel direction
		// instead and
		// only rotate by the complement

		// if (Math.abs(speed) <= MAX_SPEED){
		// if (Math.abs(deltaDegrees) > 90.0) {
		// 	deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
		// 	speed = -speed;
		// }
		// }

		double targetPosition = (currentPosition + ((deltaDegrees * ENCODER_COUNT_PER_ROTATION) / 360.0)) % ENCODER_COUNT_PER_ROTATION;
		double pidOut = pidController.calculate(currentPosition, targetPosition);
		double outVal = pidOut / ENCODER_COUNT_PER_ROTATION;

		steer.set(ControlMode.Position, outVal);
		drive.set(ControlMode.PercentOutput, speed);

		SmartDashboard.putNumber((name+"currentPosition"), currentPosition);
		SmartDashboard.putNumber((name+"Setpoint"), targetPosition);

		 SmartDashboard.putNumber((name+"PID Out: "), pidOut);
		 SmartDashboard.putNumber((name+"Out: "), outVal);
		SmartDashboard.putNumber((name+"speed: "), speed);

	}
}
