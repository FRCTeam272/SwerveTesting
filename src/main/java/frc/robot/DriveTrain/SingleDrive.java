package frc.robot.DriveTrain;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.json.JSONObject;

public class SingleDrive {
    // Swap out the motor controllers of this file where needed
    TalonFX speedMotor;
    TalonSRX turnMotor;
    
    double offset;
    double currentAngle = 0;

    /**
     * Creates objects and assigns to the ports as listed in <a href="../../deploy.config.json">config file</a>
     * @param configuration
     */
    public SingleDrive(JSONObject configuration){
        this.speedMotor = new TalonFX(configuration.getInt("SpeedMotor"));
        this.turnMotor = new TalonSRX(configuration.getInt("TurnMotor"));
        this.offset = configuration.getDouble("OffsetValue");

        if(configuration.has("TurnPID")){
            this.assignPID(configuration.getJSONObject("TurnPID"), turnMotor);
        } else {
            this.generalTurnSetup();
        }

        if(configuration.has("SpeedPID")){
            this.assignPID(configuration.getJSONObject("SpeedPID"), speedMotor);
        } else {
            this.generalSpeedSetup();
        }
    }

    private void generalSpeedSetup() {
        this.speedMotor.configNominalOutputForward(0, 100);
        this.speedMotor.configNominalOutputReverse(0, 100);
        this.speedMotor.configPeakOutputForward(1, 100);
        this.speedMotor.configPeakOutputReverse(-1, 100);
        this.speedMotor.setNeutralMode(NeutralMode.Coast);
        this.speedMotor.set(ControlMode.PercentOutput, 0.0); 
    }   

    private void generalTurnSetup() {
        //Timeout configured for 100ms to wait for configFeedbackSensor. if timeout error
	    	ErrorCode error = this.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 100);
	    	if(error.value!=0) {
	    		System.out.println("!!!!!!!! Error updating Config for Feedback Senor while configuring CAN Talon" + error.toString());
	    	}

			System.out.println("Making a SRX in PosMode");
	    		
	    	// Don't accumulate position information e.g. position is absolute
	    	this.turnMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 1, 0x00, 0x00, 0x00);
	    	
			// Choose to ensure sensor is positive when output is positive
			this.turnMotor.setSensorPhase(false);

			// Choose based on what direction you want forward/positive to be.
			// This does not affect sensor phase. 
			this.turnMotor.setInverted(false);

			// Set the peak and nominal outputs
			this.turnMotor.configNominalOutputForward(0, 100);
			this.turnMotor.configNominalOutputReverse(0, 100);
			this.turnMotor.configPeakOutputForward(1, 100);
			this.turnMotor.configPeakOutputReverse(-1, 100);
			
			this.turnMotor.setNeutralMode(NeutralMode.Brake);

			this.turnMotor.configAllowableClosedloopError(0, 0, 100);
    }

    // TODO: make this a JSON Helper class
    public static double getDoubleOrDefault(JSONObject object, String key, double fallback){
        if(object.has(key)) 
        {
            return object.getDouble(key);
        }
        return fallback;
    }

    /**
     * assigns specific pids to the SRX motor controller
     * @param config- JSON from the config object
     * @param motor
     */
    private void assignPID(JSONObject config, TalonSRX motor) {
        generalTurnSetup();
        motor.configAllowableClosedloopError(0, getDoubleOrDefault(config, "AllowableClosedLoopErr", 0), 100);

        // Set the PID loop values
        motor.config_kP(0, getDoubleOrDefault(config, "kP", 10.0), 100);
        motor.config_kI(0, getDoubleOrDefault(config, "kI", 0.0), 100);
        motor.config_kD(0, getDoubleOrDefault(config, "kD", 100.0), 100);
        motor.config_kF(0, getDoubleOrDefault(config, "kF", 0.2), 100);
    }


    /**
     * assigns specific pids to the FX motor controller
     * @param config- JSON from the config object
     * @param motor
     */
    private void assignPID(JSONObject config, TalonFX motor) {
        generalSpeedSetup();
        motor.configAllowableClosedloopError(0, getDoubleOrDefault(config, "AllowableClosedLoopErr", 0), 100);

        // Set the PID loop values
        motor.config_kP(0, getDoubleOrDefault(config, "kP", 10.0), 100);
        motor.config_kI(0, getDoubleOrDefault(config, "kI", 0.0), 100);
        motor.config_kD(0, getDoubleOrDefault(config, "kD", 100.0), 100);
        motor.config_kF(0, getDoubleOrDefault(config, "kF", 0.2), 100);
    }

    /**
     * sets the motors to new values this most likely will run at every loop
     * @param turn the expected turn angle, 
     * @param speed
     */
    public void update(double turn, double speed){
        // we can handle speed changes here if necessarily 
        this.speedMotor.set(ControlMode.PercentOutput, speed);

        turn += offset;
        turn = turn % 1024;

        if(turn < 0 + 5){
            if(currentAngle > 1024 - 10){
                this.turnMotor.set(ControlMode.PercentOutput, 1);
                this.currentAngle = this.turnMotor.getSelectedSensorPosition();
                return;
            }
        }

        if(turn > 1024 - 5){
            if(currentAngle < 0 + 10){
                this.turnMotor.set(ControlMode.PercentOutput, -1);
                this.currentAngle = this.turnMotor.getSelectedSensorPosition();
                return;
            }
        }

        currentAngle = turn;
        this.turnMotor.set(ControlMode.Position, turn);


    }
}
