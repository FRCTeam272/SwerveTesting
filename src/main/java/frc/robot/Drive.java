package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Drive extends SubsystemBase {
    private static TalonFX driveLeftFront;
    private static TalonFX driveLeftRear;
    private static TalonFX driveRightFront;
    private static TalonFX driveRightRear;

    private static TalonSRX steerLeftFront;    
    private static TalonSRX steerLeftRear;
    private static TalonSRX steerRightFront;
    private static TalonSRX steerRightRear;

    private static AnalogInput inputLeftFront;        
    private static AnalogInput inputLeftRear;
    private static AnalogInput inputRightFront;
    private static AnalogInput inputRightRear;

    public static final double WHEEL_BASE_LENGTH = 18;
    public static final double WHEEL_BASE_WIDTH = 24.5;
    public static final double ENCODER_COUNT_PER_ROTATION = 1024.0;

    // TODO: Find this value
    public static final double WHEEL_DIAMETER = 4.0;


}
