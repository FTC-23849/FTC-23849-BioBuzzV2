package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drivetrain {
    public HardwareMap hwmap;
    public DcMotorEx LF, RF, LB, RB;
    public GoBildaPinpointDriver pinpoint;
    public boolean sotm = false;
    ElapsedTime accelTimer = new ElapsedTime();
    double lastVelX = 0, lastVelY = 0;
    double accelX = 0, accelY = 0;

    public Drivetrain(HardwareMap hwmap){
        this.hwmap = hwmap;
    }

    public void driveRobot(Gamepad gamepad) {

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        LF.setPower((y + x + rx) / denominator);
        LB.setPower((y - x + rx) / denominator);
        RF.setPower((y - x - rx) / denominator);
        RB.setPower((y + x - rx) / denominator);
    }

    // Robot pushed into a corner with its back against the end wall and its side against the side wall.
    // Center is (back-to-center) from the end wall and (side-to-center) from the side wall.
    public void relocalize(String alliance, boolean leftCorner){
        double x = 72 - robotBackToCenter;
        double y = 72 - robotSideToCenter;
        if(alliance.equals("blue")) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, leftCorner ? -y : y, AngleUnit.DEGREES, 180));
        } else {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -x, leftCorner ? y : -y, AngleUnit.DEGREES, 0));
        }
    }
    public void recalibrate(){
        pinpoint.recalibrateIMU();
    }

    public Pose2D getRobotPos(){
        Pose2D pos = pinpoint.getPosition();
        if(sotm){
            double velocityX = pinpoint.getVelX(DistanceUnit.INCH);
            double velocityY = pinpoint.getVelY(DistanceUnit.INCH);
            double L = sotmLatency;
            double tof = sotmTimeOfFlight;

            double releaseX = pos.getX(DistanceUnit.INCH) + velocityX * L + 0.5 * accelX * L * L;
            double releaseY = pos.getY(DistanceUnit.INCH) + velocityY * L + 0.5 * accelY * L * L;
            double releaseVelX = velocityX + accelX * L;
            double releaseVelY = velocityY + accelY * L;

            return new Pose2D(DistanceUnit.INCH,
                    releaseX + releaseVelX * tof,
                    releaseY + releaseVelY * tof,
                    AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES));
        }else {
            return pos;
        }
    }
    public double getRobotAngle(){
        double signedAngle = pinpoint.getHeading(AngleUnit.DEGREES);
        double unsignedAngle = signedAngle;
        if (unsignedAngle < 0) {
            unsignedAngle+=360;
        }
        return  unsignedAngle;
    }

    public void init(){
        LF = hwmap.get(DcMotorEx.class, "LF");
        RF = hwmap.get(DcMotorEx.class, "RF");
        LB = hwmap.get(DcMotorEx.class, "LB");
        RB = hwmap.get(DcMotorEx.class, "RB");
        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);
        brake();
        pinpoint = hwmap.get(GoBildaPinpointDriver.class, "pinpoint");
        // x offset = forward pod's sideways distance from center (left +)
        // y offset = strafe pod's forward distance from center (forward +)
        // values from the Pedro Pinpoint tuner, keep in sync with pedro/Constants.localizerConfig
        pinpoint.setOffsets(2.800179128571758, 0.9196552516907219, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    public void brake(){
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    public void update(boolean sotm){
        this.sotm = sotm;
        pinpoint.update();

        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);
        double dt = accelTimer.seconds();
        accelTimer.reset();
        if (dt > 0) {
            accelX += sotmAccelFilter * ((velX - lastVelX) / dt - accelX);
            accelY += sotmAccelFilter * ((velY - lastVelY) / dt - accelY);
        }
        lastVelX = velX;
        lastVelY = velY;
    }
}
