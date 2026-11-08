package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    // Shot tables, "distance,velocity". Copied from v1 as a starting point, retune each unit.
    public static int numOfPointsRegressionNectar = 8;
    public static String[] shotPointsNectar = {
            "25,1040", "40,1120", "50,1200", "60,1300", "70,1350", "80,1390", "93,1440", "100,1480",
            "0,0", "0,0", "0,0", "0,0", "0,0", "0,0", "0,0", "0,0"
    };
    public static int numOfPointsRegressionPollen = 8;
    public static String[] shotPointsPollen = {
            "25,1040", "40,1120", "50,1200", "60,1300", "70,1350", "80,1390", "93,1440", "100,1480",
            "0,0", "0,0", "0,0", "0,0", "0,0", "0,0", "0,0", "0,0"
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

    // One fitted distance -> velocity curve. Re-fits only when the points or degree change.
    static class ShotTable {
        String[] parsedPoints = null;
        int parsedDegree = -1;
        PolynomialFunction curve = null;
        String error = "table not parsed yet";

        void refresh(String[] shotPoints, int numOfPoints, int maxDegree){
            int count = Math.max(0, Math.min(numOfPoints, shotPoints.length));
            String[] used = Arrays.copyOf(shotPoints, count);
            if(Arrays.equals(used, parsedPoints) && maxDegree == parsedDegree){
                return;
            }
            parsedPoints = used;
            parsedDegree = maxDegree;
            try{
                double[] dists = new double[count];
                double[] vels = new double[count];
                for (int i = 0; i < count; i++) {
                    String[] parts = used[i].split(",");
                    if(parts.length != 2){
                        throw new IllegalArgumentException("point " + i + " should be distance,velocity but is \"" + used[i] + "\"");
                    }
                    dists[i] = Double.parseDouble(parts[0].trim());
                    vels[i] = Double.parseDouble(parts[1].trim());
                }
                if(numOfPoints > count){
                    throw new IllegalArgumentException("numOfPointsRegression is " + numOfPoints + " but list only holds " + count);
                }
                if(count < 2){
                    throw new IllegalArgumentException("need at least 2 points");
                }
                WeightedObservedPoints points = new WeightedObservedPoints();
                for (int i = 0; i < dists.length; i++) {
                    points.add(dists[i], vels[i]);
                }
                int degree = Math.max(1, Math.min(maxDegree, dists.length - 1));
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
        shotTableNectar.refresh(shotPointsNectar, numOfPointsRegressionNectar, maxShotCurveDegree);
        shotTablePollen.refresh(shotPointsPollen, numOfPointsRegressionPollen, maxShotCurveDegree);
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

    // hood angle (deg) for a launch angle, clamped to what the hood can do
    double hoodAngleFor(double launchAngle){
        return Math.max(hoodMinAngle, Math.min(hoodMaxAngle, 90 - launchAngle));
    }

    // servo position for a hood angle, using that hood's endpoints
    double hoodServoPosition(double hoodAngle, double hoodUp, double hoodDown){
        double t = (hoodAngle - hoodMinAngle) / (hoodMaxAngle - hoodMinAngle);
        return hoodDown + t * (hoodUp - hoodDown);
    }

    // Distance from each turret to the goal. Pose is the robot center (field-centered inches).
    void calculateTargetDists(Pose2D pos){
        double[] goal = robot.nectarTurret.calculateGoalPos(pos);
        double[] nectarPos = robot.nectarTurret.calculateTurretPos(pos);
        double[] pollenPos = robot.pollenTurret.calculateTurretPos(pos);
        setTargets(goal, nectarPos, pollenPos);
    }

    // Same, but robotX/robotY in raw pedro coords (0-144), heading in degrees
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

    // One PIDF step for one flywheel. Returns the voltage-compensated power it sent.
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
        flywheelPIDFNectar = new PIDFController(0, kPNectar, kINectar, kDNectar, kSNectar, kVNectar);
        flywheelPIDFPollen = new PIDFController(0, kPPollen, kIPollen, kDPollen, kSPollen, kVPollen);
        voltageSensor = hwmap.voltageSensor.iterator().next();
        batteryVoltage = voltageSensor.getVoltage();
        if(batteryVoltage <= 0){
            batteryVoltage = nominalVoltage;
        }
        voltageTimer.reset();
        // TODO: check on the robot that positive power spins each flywheel the shooting way,
        //  and flip these if not. getVelocity() follows this direction, so no manual negation.
        nectarShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        pollenShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarHood.setPosition(hoodUpNectar);
        pollenHood.setPosition(hoodUpPollen);
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    // Sets both hoods from the current target distances
    void aimHoods(){
        launchAngleNectar = calculateLaunchAngle(targetDistNectar);
        launchAnglePollen = calculateLaunchAngle(targetDistPollen);
        hoodAngleNectar = hoodAngleFor(launchAngleNectar);
        hoodAnglePollen = hoodAngleFor(launchAnglePollen);
        nectarHood.setPosition(hoodServoPosition(hoodAngleNectar, hoodUpNectar, hoodDownNectar));
        pollenHood.setPosition(hoodServoPosition(hoodAnglePollen, hoodUpPollen, hoodDownPollen));
    }

    // Runs both flywheels at their target velocities, or stops them
    void runFlywheels(boolean spin, double testingVelocity){
        if(spin && (robot.testingShooter || shotTableErrorNectar == null)){
            targetVelocityNectar = robot.testingShooter ? testingVelocity
                    : Math.max(0, calculateShotSpeedNectar(targetDistNectar) + robot.powerAdjustment);
            runFlywheel(flywheelPIDFNectar, nectarShooter, nectarVelocity, targetVelocityNectar,
                    kPNectar, kINectar, kDNectar, kSNectar, kVNectar);
        }else{
            targetVelocityNectar = 0;
            stopFlywheel(flywheelPIDFNectar, nectarShooter);
        }

        if(spin && (robot.testingShooter || shotTableErrorPollen == null)){
            targetVelocityPollen = robot.testingShooter ? testingVelocity
                    : Math.max(0, calculateShotSpeedPollen(targetDistPollen) + robot.powerAdjustment);
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
            nectarHood.setPosition(robot.hoodPositionTesting);
            pollenHood.setPosition(robot.hoodPositionTesting);
        }else if(aim){
            aimHoods();
        }else{
            nectarHood.setPosition(hoodUpNectar);
            pollenHood.setPosition(hoodUpPollen);
        }

        runFlywheels(aim, robot.flywheelSpeedTesting);
    }

    // Auto: robotX/robotY in raw pedro coords (0-144), heading in degrees
    public void update(double robotX, double robotY, double headingDeg){
        refreshShotTables();
        updateBatteryVoltage();

        //target and distance are always calculated so they show in telemetry
        calculateTargetDists(robotX, robotY, headingDeg);
        readVelocities();

        if(robot.testingShooter){
            nectarHood.setPosition(robot.hoodPositionTesting);
            pollenHood.setPosition(robot.hoodPositionTesting);
        }else{
            aimHoods();
        }

        runFlywheels(true, robot.flywheelSpeedTesting - 60);
    }
}
