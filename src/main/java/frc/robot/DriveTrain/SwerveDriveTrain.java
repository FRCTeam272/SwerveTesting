package frc.robot.DriveTrain;

import org.json.JSONObject;

public class SwerveDriveTrain {
    // the distance between each wheel axle on the length and width
    // recorded in inches
    private double LENGTH;
    private double WIDTH;

    private double R;

    private SingleDrive backRight;
    private SingleDrive backLeft;
    private SingleDrive frontRight;
    private SingleDrive frontLeft;

    public SwerveDriveTrain(JSONObject config){
        this.frontRight = new SingleDrive(config.getJSONObject("FrontRight"));
        this.frontLeft = new SingleDrive(config.getJSONObject("FrontLeft"));
        this.backRight = new SingleDrive(config.getJSONObject("BackRight"));
        this.backLeft = new SingleDrive(config.getJSONObject("BackLeft"));

        this.LENGTH = config.getInt("LENGTH");
        this.WIDTH = config.getInt("WIDTH");
        this.R = Math.sqrt (
            (LENGTH * LENGTH) + 
            (WIDTH * WIDTH)
        );
    }

    private double calcSpeed(double val1, double val2){
        return Math.sqrt((val1*val1) + (val2*val2));
    }

    private double calcAngle(double val1, double val2){
        // TODO: convert this to work on units instead of angles
        var angle = Math.atan2(val1, val2) / Math.PI;
        angle = (angle / 360) * 1024;
        return angle;
    }

    /**
     * The first two parameters are the x and y axis from the strafing joystick and the last parameter is the x axis from the rotation joystick
     * @param STR : x axis from the strafing joystick
     * @param FWD : y axis from the strafing joystick
     * @param RCW : x axis from the rotation joystick
     */
    public void drive (double STR, double FWD, double RCW) {
        FWD *= -1;

        double a = STR - RCW * (LENGTH / R);
        double b = STR + RCW * (LENGTH / R);
        double c = FWD - RCW * (WIDTH / R);
        double d = FWD + RCW * (WIDTH / R);
        
        var frontRightSpeed = calcSpeed(b, c);
        var frontRightAngle = calcAngle(b,c);

        var frontLeftSpeed = calcSpeed(b,d);
        var frontLeftAngle = calcAngle(b,d);

        var backRightSpeed = calcSpeed(a,d);
        var backRightAngle = calcAngle(a,d);

        var backLeftSpeed = calcSpeed(a,c);
        var backLeftAngle = calcAngle(a, c);

        double max = 0;
        for (double val : new double[] {frontRightSpeed, backRightSpeed, frontLeftSpeed, backLeftSpeed}) {
            if(max < val){
                max = val;
            }
        }

        if(max > 1){
            frontRightSpeed /= max;
            frontLeftSpeed /= max;
            backRightSpeed /= max;
            backLeftSpeed /= max;
        }

        frontRight.update(frontRightAngle, frontRightSpeed);
        frontLeft.update(frontLeftAngle, frontLeftSpeed);
        backRight.update(backRightAngle, backRightSpeed);
        backLeft.update(backLeftAngle, backRightSpeed);
    }
}
