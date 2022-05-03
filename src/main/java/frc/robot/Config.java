package frc.robot;

public class Config {
    // TODO: @Ryan
    // Move the contents of the old config into this config as a code file
    // https://github.com/FRCTeam272/LC2022/blob/master/LC2022/src/main/deploy/Config.json
    // Shooter has been done as an example

    // USAGE:
    // import frc.robot.Config
    // Config.Shooter.ShooterMotor.Port gives us the value that we stored here

    public static class Shooter{
        public static class ShooterMotor{
            public static int port = 43;
            public static double kP = .001;
            public static double kI = 0;
            public static double kD = 0;
            public static double fIz = 0;
            public static double kFF = .00015;
            public static double kMaxOutput = 1;
            public static double kMinOutput = -1;
            public static int maxRpm = 5700;
            public static String motorType = "CANSparkMax";
        }
        public static class AimMotor {
            public static int port = 40;
            public static double kP = .001;
            public static double kI = 0;
            public static double kD = 0;
            public static double fIz = 0;
            public static double kFF = .00015;
            public static double kMaxOutput = 1;
            public static double kMinOutput = -1;
            public static int maxRpm = 5700;
            public static String motorType = "CANSparkMax";
        }
    }

    public static class Intake {
        public static class PositionControlMotor{
            public static int port = 25;
            public static double kp = 0.001;
            public static double kI = 0;
            public static double kD = 0;
            public static double kIz = 0;
            public static double kFF = 0.00015;
            public static double kMaxOutput = 1;
            public static double kMinOutput = -1;
            public static int maxRPM = 5700;
            public static String motorType = "SPARK_Max with CAN";
        }

        public static class BeltMotor{
            public static int Port = 42;
            public static double kP = 0.001;
            public static double kI = 0;
            public static double kD = 0;
            public static double kIz = 0;
            public static double kFF = 0.00015;
            public static double kMaxOutput = 1;
            public static double kMinOutput = -1;
            public static int maxRPM = 5700;
            public static String motorType = "SPARK_Max with CAN";
        }
    }

    public static class Hanger{
        public static class HangerMotor{
            public static int port = 41;
            public static double kP = 0.001;
            public static double kI = 0;
            public static double kD = 0;
            public static double kIz = 0;
            public static double kFF = 0.00015;
            public static double kMaxOutput = 1;
            public static double kMinOutput = -1;
            public static int MAXrpm = 5700;
            public static String motorType = "SPARK_Max with Can";
        }
    }

    public static class Presenter{
        public static int port = 24;
    }

    public static class DriveTrain {
        public static class FrontRight{

        }

        public static class FrontLeft{

        }

        public static class BackRight{

        }

        public static class BackLeft{

        }
    }

    public static double slowDownValue = 0.6;
}
