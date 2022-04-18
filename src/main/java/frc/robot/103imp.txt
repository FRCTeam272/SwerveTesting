/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Navx;

public class Drive extends SubsystemBase {

	private CommandScheduler scheduler;

	private static CANSparkMax driveLeftFront;
	private static CANSparkMax driveLeftRear;
	private static CANSparkMax driveRightFront;
	private static CANSparkMax driveRightRear;
	private static CANSparkMax steerLeftFront;
	private static CANSparkMax steerLeftRear;
	private static CANSparkMax steerRightFront;
	private static CANSparkMax steerRightRear;

	private static AnalogInput inputLeftFront;
	private static AnalogInput inputLeftRear;
	private static AnalogInput inputRightFront;
	private static AnalogInput inputRightRear;

	private static PIDController steerLeftFrontPID;
	private static PIDController steerLeftRearPID;
	private static PIDController steerRightFrontPID;
	private static PIDController steerRightRearPID;

	public static final double WHEEL_BASE_LENGTH = 20; // 28.0;
	public static final double WHEEL_BASE_WIDTH = 24; // 22.0;
	public static final double ENCODER_COUNT_PER_ROTATION = 4096.0;

	public static final double WHEEL_DIAMETER = 4.0;
	// TODO: increase MAX_SPEED
	public static final double MAX_SPEED = 0.3; // Max speed is 0 to 1
	public static final double STEER_DEGREES_PER_COUNT = 360.0 / ENCODER_COUNT_PER_ROTATION;
	public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
	public static final double DEADZONE = 0.1;
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

	private static final double STEER_P = .2, STEER_I = 0.00, STEER_D = 0.002;
	private static final int STATUS_FRAME_PERIOD = 5;
	private static final double RAMP_RATE = 0.5;

	public static final double OMEGA_SCALE = 1.0 / 30.0;
	private double originHeading = 0.0;
	private double originCorr = 0;
	private DoubleSupplier angleSupplier;
	private final double leftPow = 1.0;
	private final double rightPow = 1.0;

	public Drive() {
		scheduler = CommandScheduler.getInstance();
		scheduler.registerSubsystem(this);
		
		driveLeftFront = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
		driveLeftFront.restoreFactoryDefaults();
		driveLeftFront.setInverted(false);
		driveLeftFront.setOpenLoopRampRate(RAMP_RATE);

		driveLeftRear = new CANSparkMax(Constants.DRIVE_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
		driveLeftRear.restoreFactoryDefaults();
		driveLeftRear.setInverted(false);
		driveLeftRear.setOpenLoopRampRate(RAMP_RATE);

		driveRightFront = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
		driveRightFront.restoreFactoryDefaults();
		driveRightFront.setInverted(false);
		driveRightFront.setOpenLoopRampRate(RAMP_RATE);

		driveRightRear = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
		driveRightRear.restoreFactoryDefaults();
		driveRightRear.setInverted(false);
		driveRightRear.setOpenLoopRampRate(RAMP_RATE);

		inputLeftFront = new AnalogInput(Constants.DRIVE_FRONT_LEFT_STEER_ENCODER);
		steerLeftFront = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_STEER_MOTOR, MotorType.kBrushless);
		steerLeftFront.restoreFactoryDefaults();
		steerLeftFront.setInverted(false);
		steerLeftFront.setOpenLoopRampRate(RAMP_RATE);

		inputLeftRear = new AnalogInput(Constants.DRIVE_BACK_LEFT_STEER_ENCODER);
		steerLeftRear = new CANSparkMax(Constants.DRIVE_BACK_LEFT_STEER_MOTOR, MotorType.kBrushless);
		steerLeftRear.restoreFactoryDefaults();
		steerLeftRear.setInverted(false);
		steerLeftRear.setOpenLoopRampRate(RAMP_RATE);

		inputRightFront = new AnalogInput(Constants.DRIVE_FRONT_RIGHT_STEER_ENCODER);
		steerRightFront = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_STEER_MOTOR, MotorType.kBrushless);
		steerRightFront.restoreFactoryDefaults();
		steerRightFront.setInverted(false);
		steerRightFront.setOpenLoopRampRate(RAMP_RATE);

		inputRightRear = new AnalogInput(Constants.DRIVE_BACK_RIGHT_STEER_ENCODER);
		steerRightRear = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_STEER_MOTOR, MotorType.kBrushless);
		steerRightRear.restoreFactoryDefaults();
		steerRightRear.setInverted(false);
		steerRightRear.setOpenLoopRampRate(RAMP_RATE);

		steerLeftFrontPID = new PIDController(STEER_P, STEER_I, STEER_D);
		steerLeftFrontPID.enableContinuousInput(0, ENCODER_COUNT_PER_ROTATION);

		steerLeftRearPID = new PIDController(STEER_P, STEER_I, STEER_D);
		steerLeftRearPID.enableContinuousInput(0, ENCODER_COUNT_PER_ROTATION);

		steerRightFrontPID = new PIDController(STEER_P, STEER_I, STEER_D);
		steerRightFrontPID.enableContinuousInput(0, ENCODER_COUNT_PER_ROTATION);

		steerRightRearPID =  new PIDController(STEER_P, STEER_I, STEER_D);
		steerRightRearPID.enableContinuousInput(0, ENCODER_COUNT_PER_ROTATION);

		/*steerLeftFront = new TalonSRX(Constants.STEER_LEFT_FRONT_TALON);
		steerLeftFront.configFactoryDefault();
		steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog , 0, 0);
		steerLeftFront.setInverted(false);
		steerLeftFront.setSensorPhase(false);
		steerLeftFront.config_kP(0, STEER_P, 0);
		steerLeftFront.config_kI(0, STEER_I, 0);
		steerLeftFront.config_kD(0, STEER_D, 0);
		steerLeftFront.config_IntegralZone(0, 100, 0);
		steerLeftFront.configAllowableClosedloopError(0, 5, 0);
		steerLeftFront.setNeutralMode(NeutralMode.Brake);*/
		// steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
		// STATUS_FRAME_PERIOD, 0);
		
		
	}


	public double steerPID(AnalogInput encoder){
		double angle = (1.0 - encoder.getValue() / ENCODER_COUNT_PER_ROTATION) * 2.0 * Math.PI;
            //angle += offset;
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle; 
	}

	public void drive(double leftY, double leftX, double rightX, boolean fieldCentric) {
		// final double originOffset = 360 - originHeading;
		// originCorr = Constants.navX.getFusedHeading() + originOffset;

		
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

		// Add a small deadzone on the joysticks
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

		// Set each swerve module, scaling the drive speeds by the maximum speed
		setSwerveModule("LF", inputLeftFront, steerLeftFrontPID, driveLeftFront, steerLeftFront, angleLF, speedLF / maxSpeed);
		setSwerveModule("LR", inputLeftRear, steerLeftRearPID, driveLeftRear, steerLeftRear, angleLR, speedLR / maxSpeed);
		// setSwerveModule(inputRightFront, steerRightFrontPID, driveRightFront, steerRightFront, angleRF, speedRF / maxSpeed);
		setSwerveModule("RR", inputRightRear, steerRightRearPID ,driveRightRear, steerRightRear, angleRR, speedRR / maxSpeed);
	}

	private double speed(double val1, double val2) {
		return Math.hypot(val1, val2);
	}

	private double angle(double val1, double val2) {
		return Math.toDegrees(Math.atan2(val1, val2));
	}

	private void setSwerveModule(String name, AnalogInput steerAnalogIn, PIDController pidController, CANSparkMax drive, CANSparkMax steer, double angle, double speed) {
		double currentPosition = steerAnalogIn.getValue();
		
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

		steer.set(outVal);
		drive.set(speed);

		SmartDashboard.putNumber((name+"currentPosition"), currentPosition);
		SmartDashboard.putNumber((name+"Setpoint"), targetPosition);

		 SmartDashboard.putNumber((name+"PID Out: "), pidOut);
		 SmartDashboard.putNumber((name+"Out: "), outVal);
		SmartDashboard.putNumber((name+"speed: "), speed);

	}

	// get Encoder values
	public double getDriveLFEncoder() {
		return driveLeftFront.getEncoder().getPosition();
	}

	public double getDriveLREncoder() {
		return driveLeftRear.getEncoder().getPosition();
	}

	public double getDriveRFEncoder() {
		return driveRightFront.getEncoder().getPosition();
	}

	public double getDriveRREncoder() {
		return driveRightRear.getEncoder().getPosition();
	}

	public double getSteerLFEncoder() {
		return inputLeftFront.getValue();
	}

	public double getSteerLREncoder() {
		return inputLeftRear.getValue();
	}

	public double getSteerRFEncoder() {
		return inputRightFront.getValue();
	}

	public double getSteerRREncoder() {
		return inputRightRear.getValue();
	}

	// setting motors
	public void setDriveLeftFront(double speed) {
		driveLeftFront.set(speed);
	}

	public void setDriveLeftRear(double speed) {
		driveLeftRear.set(speed);
	}

	public void setDriveRightFront(double speed) {
		driveRightFront.set(speed);
	}
	public void setDriveRightRear(double speed) {
		driveRightRear.set(speed);
	}

	public void setSteerLeftFront(double speed) {
		steerLeftFront.set(speed);
	}

	public void setSteerLeftRear(double speed) {
		steerLeftRear.set(speed);
	}

	public void setSteerRightFront(double speed) {
		steerRightFront.set(speed);
	}

	public void setSteerRightRear(double speed) {
		steerRightRear.set(speed);
	}

	// public void encoderReset() {
	// 	inputRightRear.resetAccumulator();
	// 	inputRightFront.resetAccumulator();
	// 	inputLeftRear.resetAccumulator();
	// 	inputLeftFront.resetAccumulator();
	// }

	public static double[] getEncoderValues() {
		double[] values = new double[] {
			inputLeftFront.getValue(),
			inputLeftRear.getValue(),
			inputRightFront.getValue(),
			inputRightRear.getValue()
		};

		return values;
	}

	public static double[] getEncoderVal() {
		double[] values = new double[] {
			inputLeftFront.pidGet() ,
			inputLeftRear.pidGet(),
			inputRightFront.pidGet(),
			inputRightRear.pidGet()
		};

		return values;
	}
	public void initDefaultCommand() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
