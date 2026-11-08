package org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.subsystems.Misc;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Turret;

public class Shooter {

    private final org.firstinspires.ftc.teamcode.opmode.subsystems.Shooter shooter;
    private final Turret nectarTurret, pollenTurret;
    private final Misc misc;

    // Uses the robot's own shooter and turrets, since the shooter needs the turrets'
    // positions to work out each shooter's distance to the goal
    public Shooter(HardwareMap hwmap, Robot robot) {
        this.shooter = robot.shooter;
        this.nectarTurret = robot.nectarTurret;
        this.pollenTurret = robot.pollenTurret;
        this.misc = new Misc(hwmap);
    }

    public void init() {
        shooter.init();
        nectarTurret.init();
        pollenTurret.init();
        misc.init();
        misc.start();
    }

    public boolean canShoot(String alliance) {

        return misc.canShoot(alliance);

    }

    public double getNectarMotorSpeed() {
        return shooter.nectarShooter.getVelocity();
    }

    public double getPollenMotorSpeed() {
        return shooter.pollenShooter.getVelocity();
    }

    public double getNectarMotorPower() {
        return shooter.nectarShooter.getPower();
    }

    public double getPollenMotorPower() {
        return shooter.pollenShooter.getPower();
    }

    // Same distances the flywheels/hoods use (robotX/robotY raw pedro coords, heading in degrees)
    public double getNectarTargetDistance(double robotX, double robotY, double headingDeg) {
        shooter.calculateTargetDists(robotX, robotY, headingDeg);
        return shooter.targetDistNectar;
    }

    public double getPollenTargetDistance(double robotX, double robotY, double headingDeg) {
        shooter.calculateTargetDists(robotX, robotY, headingDeg);
        return shooter.targetDistPollen;
    }

    // Same velocity lookups the flywheels use for a given distance
    public double getNectarTargetVelocity(double targetDistance) {
        return shooter.calculateShotSpeedNectar(targetDistance);
    }

    public double getPollenTargetVelocity(double targetDistance) {
        return shooter.calculateShotSpeedPollen(targetDistance);
    }

    public int getShotZone() {
        return shooter.shotZone + 1;
    }

    public String getShotTableErrorNectar() {
        return shooter.shotTableErrorNectar;
    }

    public String getShotTableErrorPollen() {
        return shooter.shotTableErrorPollen;
    }

    public double getNectarHoodPosition() {
        return shooter.nectarHood.getPosition();
    }

    public double getPollenHoodPosition() {
        return shooter.pollenHood.getPosition();
    }

    public double getNectarTurretServoPosition() {
        return nectarTurret.frontTurret.getPosition();
    }

    public double getPollenTurretServoPosition() {
        return pollenTurret.frontTurret.getPosition();
    }

    // Goal the turrets are aiming at (raw field coords in)
    public double[] getTurretGoal(double robotY) {
        return nectarTurret.calculateGoalPos(robotY);
    }

    public void updateTurret(double robotX, double robotY, double robotAngleUnsigned) {
        nectarTurret.update(robotX, robotY, robotAngleUnsigned);
        pollenTurret.update(robotX, robotY, robotAngleUnsigned);
    }

    public void updateFlywheel(double robotX, double robotY, double headingDeg) {
        shooter.update(robotX, robotY, headingDeg);
    }
}
