package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;

@Config
public class Shooter {
    public HardwareMap hwmap;
    public DcMotorEx nectarShooter, pollenShooter;
    public ServoImplEx nectarHood, pollenHood, stopper;
    Robot robot;

    public static String[] shotPointsNectar = {
            "20,1624", "30,1706", "40,1820", "50,1940", "60,2090", "70,2160", "80,2230", "90,2314", "100,2380",
            "0,0", "0,0", "0,0", "0,0", "0,0", "0,0", "0,0"
    };
    public static String[] shotPointsPollen = {
            "20,1304", "30,1366", "40,1420", "50,1500", "60,1600", "70,1650", "80,1690", "90,1744", "100,1780",
            "0,0", "0,0", "0,0", "0,0", "0,0", "0,0", "0,0"
    };
    public static int maxShotCurveDegree = 3;

    ShotTable shotTableNectar = new ShotTable();
    ShotTable shotTablePollen = new ShotTable();
    public String shotTableErrorNectar = "table not parsed yet";
    public String shotTableErrorPollen = "table not parsed yet";
    public int shotZone = 0;

    PIDFController flywheelPIDFNectar, flywheelPIDFPollen;

    public double targetXNectar = 0, targetYNectar = 0, targetDistNectar = 0;
    public double targetXPollen = 0, targetYPollen = 0, targetDistPollen = 0;
    public double nectarVelocity = 0, pollenVelocity = 0;
    public double targetVelocityNectar = 0, targetVelocityPollen = 0;
    public double velocityErrorNectar = 0, velocityErrorPollen = 0;
    public double launchAngleNectar = 0, launchAnglePollen = 0;
    public double hoodAngleNectar = 0, hoodAnglePollen = 0;

    VoltageSensor voltageSensor;
    ElapsedTime voltageTimer = new ElapsedTime();
    public double batteryVoltage = nominalVoltage;

    static class ShotTable {
        String[] parsedPoints = null;
        int parsedDegree = -1;
        PolynomialFunction curve = null;
        String error = "table not parsed yet";

        void refresh(String[] shotPoints, int maxDegree){
            if(Arrays.equals(shotPoints, parsedPoints) && maxDegree == parsedDegree){
                return;
            }
            parsedPoints = shotPoints.clone();
            parsedDegree = maxDegree;
            try{
                WeightedObservedPoints points = new WeightedObservedPoints();
                int count = 0;
                for (int i = 0; i < shotPoints.length; i++) {
                    String[] parts = shotPoints[i].split(",");
                    if(parts.length != 2){
                        throw new IllegalArgumentException("point " + i + " should be distance,velocity but is \"" + shotPoints[i] + "\"");
                    }
                    double dist = Double.parseDouble(parts[0].trim());
                    double vel = Double.parseDouble(parts[1].trim());
                    //skip empty 0,0 slots
                    if(dist == 0 && vel == 0){
                        continue;
                    }
                    points.add(dist, vel);
                    count++;
                }
                if(count < 2){
                    throw new IllegalArgumentException("need at least 2 points");
                }
                int degree = Math.max(1, Math.min(maxDegree, count - 1));
                curve = new PolynomialFunction(PolynomialCurveFitter.create(degree).fit(points.toList()));
                error = null;
            }catch(Exception e){
                curve = null;
                error = e.getMessage();
            }
        }

        double speed(double distanceFromTarget){
            if(error != null){
                return 0;
            }
            return Math.max(0, curve.value(distanceFromTarget));
        }
    }

    void updateBatteryVoltage(){
        if(voltageTimer.milliseconds() > 250){
            double reading = voltageSensor.getVoltage();
            if(reading > 0){
                batteryVoltage = 0.8 * batteryVoltage + 0.2 * reading;
            }
            voltageTimer.reset();
        }
    }

    double compensateVoltage(double power){
        return Math.max(-1.0, Math.min(1.0, power * nominalVoltage / batteryVoltage));
    }

    void refreshShotTables(){
        shotTableNectar.refresh(shotPointsNectar, maxShotCurveDegree);
        shotTablePollen.refresh(shotPointsPollen, maxShotCurveDegree);
        shotTableErrorNectar = shotTableNectar.error;
        shotTableErrorPollen = shotTablePollen.error;
    }

    public double calculateShotSpeedNectar(double distanceFromTarget){
        return shotTableNectar.speed(distanceFromTarget);
    }

    public double calculateShotSpeedPollen(double distanceFromTarget){
        return shotTablePollen.speed(distanceFromTarget);
    }

    double entryLaunchAngle(double d){
        if(d <= 0){
            return 90 - hoodMinAngle;
        }
        double h = hiveHeight - shooterHeight;
        double slope = 2 * h / d - Math.tan(Math.toRadians(entryAngle));
        return Math.toDegrees(Math.atan(slope));
    }

    public double calculateLaunchAngle(double distanceFromTarget){
        double d = distanceFromTarget;
        if(d >= hoodLogicDistThreshold || hoodLogicDistThreshold <= closeShotDist){
            return entryLaunchAngle(d);
        }
        double thresholdAngle = entryLaunchAngle(hoodLogicDistThreshold);
        double t = (d - closeShotDist) / (hoodLogicDistThreshold - closeShotDist);
        return closeLaunchAngle + (thresholdAngle - closeLaunchAngle) * t;
    }

    double hoodAngleFor(double launchAngle){
        return Math.max(hoodMinAngle, Math.min(hoodMaxAngle, 90 - launchAngle));
    }

    double hoodServoPosition(double hoodAngle, double hoodUp, double hoodDown){
        double t = (hoodAngle - hoodMinAngle) / (hoodMaxAngle - hoodMinAngle);
        return hoodDown + t * (hoodUp - hoodDown);
    }

    void calculateTargetDists(Pose2D pos){
        double[] goal = robot.nectarTurret.calculateGoalPos(pos);
        double[] nectarPos = robot.nectarTurret.calculateTurretPos(pos);
        double[] pollenPos = robot.pollenTurret.calculateTurretPos(pos);
        setTargets(goal, nectarPos, pollenPos);
    }

    public void calculateTargetDists(double robotX, double robotY, double headingDeg){
        double[] goal = robot.nectarTurret.calculateGoalPos(robotY);
        double[] nectarPos = robot.nectarTurret.calculateTurretPos(robotX - 72, robotY - 72, headingDeg);
        double[] pollenPos = robot.pollenTurret.calculateTurretPos(robotX - 72, robotY - 72, headingDeg);
        setTargets(goal, nectarPos, pollenPos);
    }

    void setTargets(double[] goal, double[] nectarPos, double[] pollenPos){
        targetXNectar = goal[0];
        targetYNectar = goal[1];
        targetXPollen = goal[0];
        targetYPollen = goal[1];
        targetDistNectar = Math.hypot(goal[0] - nectarPos[0], goal[1] - nectarPos[1]);
        targetDistPollen = Math.hypot(goal[0] - pollenPos[0], goal[1] - pollenPos[1]);
    }

    double runFlywheel(PIDFController pid, DcMotorEx motor, double currentVelocity, double targetVelocity,
                       double kP, double kI, double kD, double kS, double kV){
        //re-read gains every loop so FTC Dashboard edits apply live
        pid.setPID(kP, kI, kD);
        pid.setFeedforward(kS, kV);
        pid.setMaxIntegralPower(maxIntegralPower);
        pid.setTargetVelocity(targetVelocity);
        double power = compensateVoltage(pid.update(currentVelocity));
        motor.setPower(power);
        return power;
    }

    void stopFlywheel(PIDFController pid, DcMotorEx motor){
        pid.reset();
        pid.setTargetVelocity(0);
        motor.setPower(0);
    }

    public Shooter(HardwareMap hwmap, Robot robot){
        this.hwmap = hwmap;
        this.robot = robot;
    }

    public void release(){
        stopper.setPosition(releaseBall);
    }

    public void block(){
        stopper.setPosition(blockBall);
    }

    public void init(){
        nectarShooter = hwmap.get(DcMotorEx.class, "nectarShooter");
        pollenShooter = hwmap.get(DcMotorEx.class, "pollenShooter");
        nectarHood = hwmap.get(ServoImplEx.class, "nectarHood");
        pollenHood = hwmap.get(ServoImplEx.class, "pollenHood");
        stopper = hwmap.get(ServoImplEx.class, "stopper");
        nectarHood.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pollenHood.setPwmRange(new PwmControl.PwmRange(500, 2500));
        stopper.setPwmRange(new PwmControl.PwmRange(500, 2500));
        flywheelPIDFNectar = new PIDFController(0, kPNectar, kINectar, kDNectar, kSNectar, kVNectar);
        flywheelPIDFPollen = new PIDFController(0, kPPollen, kIPollen, kDPollen, kSPollen, kVPollen);
        voltageSensor = hwmap.voltageSensor.iterator().next();
        batteryVoltage = voltageSensor.getVoltage();
        if(batteryVoltage <= 0){
            batteryVoltage = nominalVoltage;
        }
        voltageTimer.reset();
        nectarShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        nectarHood.setDirection(Servo.Direction.REVERSE);
        nectarHood.setPosition(hoodUpNectar);
        pollenHood.setPosition(hoodUpPollen);
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    //linear hood: up pos at min dist, down pos at max dist, clamped
    double linearHoodPosition(double dist, double hoodUp, double hoodDown){
        double t = (dist - hoodLinearMinDist) / (hoodLinearMaxDist - hoodLinearMinDist);
        t = Math.max(0, Math.min(1, t));
        double pos = hoodUp + t * (hoodDown - hoodUp);
        return Math.max(Math.min(hoodUp, hoodDown), Math.min(Math.max(hoodUp, hoodDown), pos));
    }

    void aimHoods(){
        //old angle based hood logic
        //launchAngleNectar = calculateLaunchAngle(targetDistNectar);
        //launchAnglePollen = calculateLaunchAngle(targetDistPollen);
        //hoodAngleNectar = hoodAngleFor(launchAngleNectar);
        //hoodAnglePollen = hoodAngleFor(launchAnglePollen);
        //nectarHood.setPosition(hoodServoPosition(hoodAngleNectar, hoodUpNectar, hoodDownNectar) - 0.07);
        //pollenHood.setPosition(hoodServoPosition(hoodAnglePollen, hoodUpPollen, hoodDownPollen));

        nectarHood.setPosition(linearHoodPosition(targetDistNectar, hoodUpNectar, hoodDownNectar));
        pollenHood.setPosition(linearHoodPosition(targetDistPollen, hoodUpPollen, hoodDownPollen));
    }

    void runFlywheels(boolean spin, double testingVelocityNectar, double testingVelocityPollen){
        if(spin && (robot.testingShooter || shotTableErrorNectar == null)){
            targetVelocityNectar = robot.testingShooter ? testingVelocityNectar
                    : Math.max(0, calculateShotSpeedNectar(targetDistNectar) + robot.powerAdjustmentNectar);
            runFlywheel(flywheelPIDFNectar, nectarShooter, nectarVelocity, targetVelocityNectar,
                    kPNectar, kINectar, kDNectar, kSNectar, kVNectar);
        }else{
            targetVelocityNectar = 0;
            stopFlywheel(flywheelPIDFNectar, nectarShooter);
        }

        if(spin && (robot.testingShooter || shotTableErrorPollen == null)){
            targetVelocityPollen = robot.testingShooter ? testingVelocityPollen
                    : Math.max(0, calculateShotSpeedPollen(targetDistPollen) + robot.powerAdjustmentPollen);
            runFlywheel(flywheelPIDFPollen, pollenShooter, pollenVelocity, targetVelocityPollen,
                    kPPollen, kIPollen, kDPollen, kSPollen, kVPollen);
        }else{
            targetVelocityPollen = 0;
            stopFlywheel(flywheelPIDFPollen, pollenShooter);
        }

        velocityErrorNectar = nectarVelocity - targetVelocityNectar;
        velocityErrorPollen = pollenVelocity - targetVelocityPollen;
    }

    void readVelocities(){
        nectarVelocity = nectarShooter.getVelocity();
        pollenVelocity = pollenShooter.getVelocity();
    }

    public void update(boolean aim){
        refreshShotTables();
        updateBatteryVoltage();

        //target and distance are always calculated so they show in telemetry
        calculateTargetDists(robot.dt.getRobotPos());
        readVelocities();

        if(robot.testingShooter){
            nectarHood.setPosition(robot.hoodPositionTestingNectar);
            pollenHood.setPosition(robot.hoodPositionTestingPollen);
        }else if(aim){
            aimHoods();
        }else{
            nectarHood.setPosition(hoodUpNectar);
            pollenHood.setPosition(hoodUpPollen);
        }

        runFlywheels(aim, robot.flywheelSpeedTestingNectar, robot.flywheelSpeedTestingPollen);
    }

    public void update(double robotX, double robotY, double headingDeg){
        refreshShotTables();
        updateBatteryVoltage();

        //target and distance are always calculated so they show in telemetry
        calculateTargetDists(robotX, robotY, headingDeg);
        readVelocities();

        if(robot.testingShooter){
            nectarHood.setPosition(robot.hoodPositionTestingNectar);
            pollenHood.setPosition(robot.hoodPositionTestingPollen);
        }else{
            aimHoods();
        }

        runFlywheels(true, robot.flywheelSpeedTestingNectar - 60, robot.flywheelSpeedTestingPollen - 60);
    }
}
