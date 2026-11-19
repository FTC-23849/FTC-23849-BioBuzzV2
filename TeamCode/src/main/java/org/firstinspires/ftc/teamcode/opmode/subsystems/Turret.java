package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret {
    public ServoImplEx frontTurret, backTurret;
    Robot robot;
    public final boolean isNectar;
    public final double offsetFromCenter;

    boolean aimTurret = false;
    double turretPos = 0;
    double targetX = 0;
    double targetY = 0;
    public double turretX = 0;
    public double turretY = 0;
    public double turretAngle = 0;

    public Turret(Robot robot, double offsetFromCenter, ServoImplEx frontTurret, ServoImplEx backTurret, boolean isNectar){
        this.robot = robot;
        this.offsetFromCenter = offsetFromCenter;
        this.frontTurret = frontTurret;
        this.backTurret = backTurret;
        this.isNectar = isNectar;
    }

    double frontCenterPos(){ return isNectar ? turretFrontCenterPosNectar : turretFrontCenterPosPollen; }
    double servoDifference(){ return isNectar ? turretServoDifferenceNectar : turretServoDifferencePollen; }
    double trim(){ return isNectar ? turretTrimNectar : turretTrimPollen; }
    double rightLimit(){ return isNectar ? rightLimitNectar : rightLimitPollen; }
    double leftLimit(){ return isNectar ? leftLimitNectar : leftLimitPollen; }

    public void init(){
        frontTurret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        backTurret.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    public double[] calculateTurretPos(double centerX, double centerY, double headingDeg){
        double h = Math.toRadians(headingDeg);
        return new double[]{
                centerX + offsetFromCenter * Math.cos(h),
                centerY + offsetFromCenter * Math.sin(h)
        };
    }

    public double[] calculateTurretPos(Pose2D pos){
        return calculateTurretPos(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
    }

    public double[] calculateGoalPos(Pose2D pos){
        return chooseGoal(pos.getY(DistanceUnit.INCH));
    }

    public double[] calculateGoalPos(double robotY){
        return chooseGoal(robotY - 72);
    }

    //chooses target by picking the cell to aim at, and then
    // picking a point in the hive cell with best angle
    double[] chooseGoal(double robotY){
        if(robot.alliance.equals("blue")){
            if(robotY < 0){
                //bottom right
                return new double[]{13.1, -19.2};
            }else{
                //top right
                return new double[]{13.1, 19.2};
            }
        }else{
            if(robotY < 0){
                //bottom left
                return new double[]{-13.1, -19.2};
            }else{
                //top left
                return new double[]{-13.1, 19.2};
            }
        }
    }

    double aimServoPosition(double turretX, double turretY, double robotAngleUnsigned, double[] goal){
        this.turretX = turretX;
        this.turretY = turretY;
        targetX = goal[0];
        targetY = goal[1];

        turretAngle = Math.toDegrees(Math.atan2(targetY - turretY, targetX - turretX));
        if(turretAngle < 0){
            turretAngle+=360;
        }
        turretAngle = ((turretAngle - robotAngleUnsigned) % 360 + 360) % 360;

        double signedAngle = turretAngle > 180 ? turretAngle - 360 : turretAngle;
        double servoPosition = frontCenterPos() - signedAngle/turretRangeDeg;

        if (servoPosition < 0){
            servoPosition += 360 / turretRangeDeg;
        }
        if (servoPosition > 1){
            servoPosition -= 360 / turretRangeDeg;
        }
        return servoPosition;
    }

    public double calculateTurretServoPositionAuto(double robotX, double robotY, double robotAngleUnsigned){
        double[] goal = calculateGoalPos(robotY);
        double[] turret = calculateTurretPos(robotX - 72, robotY - 72, robotAngleUnsigned);
        double servoPosition = aimServoPosition(turret[0], turret[1], robotAngleUnsigned, goal);
        return Math.max(rightLimit(), Math.min(servoPosition, leftLimit()));
    }

    public double calculateTurretServoPosition(Pose2D pos, double robotAngleUnsigned){
        double[] goal = calculateGoalPos(pos);
        double[] turret = calculateTurretPos(pos);
        double servoPosition = aimServoPosition(turret[0], turret[1], robotAngleUnsigned, goal);
        servoPosition += trim();
        servoPosition += isNectar ? robot.turretAdjustmentNectar : robot.turretAdjustmentPollen;

        return Math.max(rightLimit(), Math.min(servoPosition, leftLimit()));
    }

    void setTurret(double position){
        turretPos = position;
        frontTurret.setPosition(position);
        backTurret.setPosition(position + servoDifference());
    }

    public void update(boolean aim){
        aimTurret = aim;
        if(aimTurret && robot.testingTurret){
            setTurret(isNectar ? robot.turretPositionTestingNectar : robot.turretPositionTestingPollen);
        }else if(aimTurret){
            setTurret(calculateTurretServoPosition(robot.dt.getRobotPos(), robot.dt.getRobotAngle()));
        }else{
            //turret is centered, so it points straight forward relative to the robot
            turretAngle = 0;
            setTurret(frontCenterPos());
        }
    }

    public void update(double robotX, double robotY, double robotAngleUnsigned){
        aimTurret = true;
        if(robot.testingTurret){
            setTurret(isNectar ? robot.turretPositionTestingNectar : robot.turretPositionTestingPollen);
        }else{
            setTurret(calculateTurretServoPositionAuto(robotX, robotY, robotAngleUnsigned));
        }
    }
}
