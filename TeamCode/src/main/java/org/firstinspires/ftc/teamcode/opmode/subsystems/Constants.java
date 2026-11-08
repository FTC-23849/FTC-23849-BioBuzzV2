package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Drivetrain
    public static double sotmLatency = 0.1;
    public static double sotmTimeOfFlight = 0.5;
    public static double sotmAccelFilter = 0.3;

    // Intake
    public static double transferSpeed = 1.0;
    public static double transferSpeedAuto = 1.0;
    public static double
            intakePower = 1.000;
    public static double reverseIntakePower = -1.000;
    public static double flowerIntakeDown = 0.71;
    public static double flowerIntakeUp = 0.97;
    // Misc

    // Shooter (values copied from v1 as a starting point, retune each unit)
    public static double blockBall = 0.510;
    public static double releaseBall = 0.380;

    public static double kPNectar = 0.003;
    public static double kINectar = 0.0;
    public static double kDNectar = 0.0;
    public static double kSNectar = 0.15;
    public static double kVNectar = 0.00030;

    public static double kPPollen = 0.003;
    public static double kIPollen = 0.0;
    public static double kDPollen = 0.0;
    public static double kSPollen = 0.15;
    public static double kVPollen = 0.00030;

    public static double nominalVoltage = 13.2;
    public static double maxIntegralPower = 0.2;

    public static double hoodUpNectar = 0.25;
    public static double hoodDownNectar = 0.43;
    public static double hoodUpPollen = 0.25;
    public static double hoodDownPollen = 0.43;
    public static double hoodMinAngle = 10;
    public static double hoodMaxAngle = 35;

    public static double shooterHeight = 369 / 25.4;
    public static double hiveHeight = 60;
    public static double entryAngle = -30;
    public static double hoodLogicDistThreshold = 60;
    public static double closeShotDist = 25;
    public static double closeLaunchAngle = 70.1;


    // Turret
    // Distance (inches) from the robot center to each turret's pivot, along the robot's
    // front/back axis. Positive = in front of center, negative = behind. MEASURE THESE.
    public static double nectarTurretOffset = 0;
    public static double pollenTurretOffset = 0;

    // Per-turret servo calibration (values copied from v1 as a starting point)
    public static double turretBackCenterPosNectar = 0.456;
    public static double turretBackCenterPosPollen = 0.456;
    public static double turretServoDifferenceNectar = 0.017;
    public static double turretServoDifferencePollen = 0.017;
    // Servo position trim added after aiming (this was turretOffset in v1)
    public static double turretTrimNectar = 0;
    public static double turretTrimPollen = 0;
    public static double rightLimitNectar = 0;
    public static double leftLimitNectar = 1;
    public static double rightLimitPollen = 0;
    public static double leftLimitPollen = 1;

    public static double turretAdjustmentStep = 0.01;
    public static double powerAdjustmentStep = 10;
    public static double turretRangeDeg = 360 * (60/20) * (35.0/99);
}
