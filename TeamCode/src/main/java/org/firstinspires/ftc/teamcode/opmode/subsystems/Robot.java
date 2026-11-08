package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {
    public HardwareMap hwmap;
    public Telemetry telemetry;
    public Drivetrain dt;
    public Intake intake;
    public Misc misc;
    public Shooter shooter;
    public Turret nectarTurret, pollenTurret;
    public List<LynxModule> hubs;
    ElapsedTime loopTimer = new ElapsedTime();
    double loopTime = 0;

    public String alliance;
    public boolean aim;
    public Robot(HardwareMap hwmap, Telemetry telemetry){
        this.hwmap = hwmap;
        this.telemetry = telemetry;
        createSubsystems();
    }

    public Robot(HardwareMap hwmap){
        this.hwmap = hwmap;
        createSubsystems();
    }

    void createSubsystems(){
        dt = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        misc = new Misc(hwmap, this);
        shooter = new Shooter(hwmap, this);
        nectarTurret = new Turret(this, nectarTurretOffset,
                hwmap.get(ServoImplEx.class, "frontTurretNectar"),
                hwmap.get(ServoImplEx.class, "backTurretNectar"),
                true);
        pollenTurret = new Turret(this, pollenTurretOffset,
                hwmap.get(ServoImplEx.class, "frontTurretPollen"),
                hwmap.get(ServoImplEx.class, "backTurretPollen"),
                false);
    }

    public void init(){
        hubs = hwmap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        dt.init();
        intake.init();
        misc.init();
        shooter.init();
        nectarTurret.init();
        pollenTurret.init();
    }

    public void initLoop(){
        dt.initLoop();
        intake.initLoop();
        misc.initLoop();
        shooter.initLoop();
        nectarTurret.initLoop();
        pollenTurret.initLoop();
    }

    public void start(){
        dt.start();
        intake.start();
        misc.start();
        shooter.start();
        nectarTurret.start();
        pollenTurret.start();
    }

    public void stop(){
        dt.stop();
        intake.stop();
        misc.stop();
        shooter.stop();
        nectarTurret.stop();
        pollenTurret.stop();
    }

    public boolean testingShooter = false;
    public boolean testingTurret = false;
    public double hoodPositionTesting = hoodDownNectar;
    public double turretPositionTesting = 0;
    public double flywheelSpeedTesting = 0;
    public double turretAdjustment = 0;
    public double powerAdjustment = 0;

    public void update(boolean aim, boolean sotm, String Alliance){
        loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        alliance = Alliance;
        this.aim = aim;
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        dt.update(sotm);
        intake.update();
        misc.update();
        shooter.update(aim);
        nectarTurret.update(aim);
        pollenTurret.update(aim);
    }

    public void addTelemetry(){
        telemetry.addData("Loop time (ms)", loopTime);
        telemetry.addData("Power Adjustment", powerAdjustment);
        telemetry.addData("Turret Adjustment", turretAdjustment);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Aim", aim);
        telemetry.addData("Robot x", dt.getRobotPos().getX(DistanceUnit.INCH));
        telemetry.addData("Robot y", dt.getRobotPos().getY(DistanceUnit.INCH));
        telemetry.addData("Robot heading (deg)", dt.getRobotAngle());
        telemetry.addData("Pinpoint encoder X", dt.pinpoint.getEncoderX());
        telemetry.addData("Pinpoint encoder Y", dt.pinpoint.getEncoderY());
        telemetry.addData("Battery voltage", shooter.batteryVoltage);

        telemetry.addLine("--- Nectar ---");
        telemetry.addData("Nectar turret pos (x, y)", "%.1f, %.1f", nectarTurret.turretX, nectarTurret.turretY);
        telemetry.addData("Nectar target (x, y)", "%.1f, %.1f", shooter.targetXNectar, shooter.targetYNectar);
        telemetry.addData("Nectar distance from target", shooter.targetDistNectar);
        telemetry.addData("Nectar flywheel velocity", shooter.nectarVelocity);
        telemetry.addData("Nectar flywheel target velocity", shooter.targetVelocityNectar);
        telemetry.addData("Nectar flywheel error (curr - target)", shooter.velocityErrorNectar);
        telemetry.addData("Nectar flywheel power", shooter.nectarShooter.getPower());
        telemetry.addData("Nectar turret angle (deg)", nectarTurret.turretAngle);
        telemetry.addData("Nectar launch angle (deg)", shooter.launchAngleNectar);
        telemetry.addData("Nectar hood angle (deg)", shooter.hoodAngleNectar);
        telemetry.addData("Nectar shot table error", shooter.shotTableErrorNectar);
        telemetry.addData("Nectar front turret servo pos", nectarTurret.frontTurret.getPosition());
        telemetry.addData("Nectar back turret servo pos", nectarTurret.backTurret.getPosition());

        telemetry.addLine("--- Pollen ---");
        telemetry.addData("Pollen turret pos (x, y)", "%.1f, %.1f", pollenTurret.turretX, pollenTurret.turretY);
        telemetry.addData("Pollen target (x, y)", "%.1f, %.1f", shooter.targetXPollen, shooter.targetYPollen);
        telemetry.addData("Pollen distance from target", shooter.targetDistPollen);
        telemetry.addData("Pollen flywheel velocity", shooter.pollenVelocity);
        telemetry.addData("Pollen flywheel target velocity", shooter.targetVelocityPollen);
        telemetry.addData("Pollen flywheel error (curr - target)", shooter.velocityErrorPollen);
        telemetry.addData("Pollen flywheel power", shooter.pollenShooter.getPower());
        telemetry.addData("Pollen turret angle (deg)", pollenTurret.turretAngle);
        telemetry.addData("Pollen launch angle (deg)", shooter.launchAnglePollen);
        telemetry.addData("Pollen hood angle (deg)", shooter.hoodAnglePollen);
        telemetry.addData("Pollen shot table error", shooter.shotTableErrorPollen);
        telemetry.addData("Pollen front turret servo pos", pollenTurret.frontTurret.getPosition());
        telemetry.addData("Pollen back turret servo pos", pollenTurret.backTurret.getPosition());
    }
}
