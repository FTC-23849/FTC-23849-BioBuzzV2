package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.powerAdjustmentStep;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.turretAdjustmentStep;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.turretFrontCenterPosNectar;

import org.firstinspires.ftc.teamcode.opmode.subsystems.Misc;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;
//testing commit
@Config
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private Follower follower;
    public static double turretPositionTesting = turretFrontCenterPosNectar;
    public static double flywheelSpeedTesting = 1200;
    public static double hoodPositionTesting = 0.25;
    public static boolean showServoStates = false;
    public static boolean showCurrentDraw = false;
    //gp2 dpad left/right or dashboard
    public static boolean testingShooter = false;
    public static boolean testingTurret = false;
    public static double turretAdjustmentPollen = 0;
    public static double powerAdjustmentPollen = 0;
    public static double turretAdjustmentNectar = 0;
    public static double powerAdjustmentNectar = 0;
    boolean aim = false;
    boolean sotm = false;
    String alliance = "red";
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        follower = org.firstinspires.ftc.teamcode.opmode.Auto.pedro.Constants.create(hardwareMap);
        robot.dt.brake();
    }

    @Override
    public void init_loop() {
        robot.initLoop();
        if(gamepad2.dpad_left){
            robot.misc.logging = false;
        }
        telemetry.addData("Current logging", robot.misc.logging ? "ON (gamepad2 dpad left to disable)" : "OFF");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
        robot.misc.stopLimelight();
        robot.misc.startLogging();
    }

    @Override
    public void loop() {
        //pollen tuning
        if(gamepad2.startWasReleased()){
            turretAdjustmentPollen += turretAdjustmentStep;
        }
        if(gamepad2.backWasReleased()){
            turretAdjustmentPollen -= turretAdjustmentStep;
        }
        if(gamepad2.dpadUpWasReleased()){
            powerAdjustmentPollen += powerAdjustmentStep;
        }
        if(gamepad2.dpadDownWasReleased()){
            powerAdjustmentPollen -= powerAdjustmentStep;
        }
        if(gamepad2.left_bumper){
            //reset pollen power
            powerAdjustmentPollen = 0;
        }
        if(gamepad2.left_trigger > 0.7){
            //reset pollen turret
            turretAdjustmentPollen = 0;
        }

        //nectar tuning
        if(gamepad2.xWasReleased()){
            //turret left
            turretAdjustmentNectar += turretAdjustmentStep;
        }
        if(gamepad2.bWasReleased()){
            //turret right
            turretAdjustmentNectar -= turretAdjustmentStep;
        }
        if(gamepad2.yWasReleased()){
            powerAdjustmentNectar += powerAdjustmentStep;
        }
        if(gamepad2.aWasReleased()){
            powerAdjustmentNectar -= powerAdjustmentStep;
        }
        if(gamepad2.right_bumper){
            //reset nectar power
            powerAdjustmentNectar = 0;
        }
        if(gamepad2.right_trigger > 0.7){
            //reset nectar turret
            turretAdjustmentNectar = 0;
        }

        //locks
        if(gamepad2.dpadLeftWasReleased()){
            //lock turret
            testingTurret = !testingTurret;
        }
        if(gamepad2.dpadRightWasReleased()){
            //lock flywheel and hood
            testingShooter = !testingShooter;
        }

        robot.turretAdjustmentPollen = turretAdjustmentPollen;
        robot.powerAdjustmentPollen = powerAdjustmentPollen;
        robot.turretAdjustmentNectar = turretAdjustmentNectar;
        robot.powerAdjustmentNectar = powerAdjustmentNectar;
        robot.testingShooter = testingShooter;
        robot.testingTurret = testingTurret;
        robot.hoodPositionTesting = hoodPositionTesting;
        robot.turretPositionTesting = turretPositionTesting;
        robot.flywheelSpeedTesting = flywheelSpeedTesting;
        robot.misc.showCurrentDraw = showCurrentDraw;
        robot.update(aim, sotm, alliance);
        robot.addTelemetry();
        telemetry.addData("cfg flywheelSpeedTesting", flywheelSpeedTesting);
        telemetry.addData("Turret locked", testingTurret);
        telemetry.addData("Flywheel locked", testingShooter);
        if(showServoStates){
            telemetry.addData("Flower intake servo pos", robot.intake.flowerIntake.getPosition());
            telemetry.addData("Nectar hood servo pos", robot.shooter.nectarHood.getPosition());
            telemetry.addData("Pollen hood servo pos", robot.shooter.pollenHood.getPosition());
            telemetry.addData("Stopper servo pos", robot.shooter.stopper.getPosition());
        }
        telemetry.addData("Current log", robot.misc.logStatus);
        telemetry.addData("Intake current (A)", robot.misc.intakeCurrent);
        telemetry.addData("Intake full", robot.misc.intakeFull);
        telemetry.addData("LED", robot.misc.led == null ? "not in config (name it light)" : robot.misc.led.getPosition());
        if(showCurrentDraw){
            for (int i = 0; i < Misc.MOTOR_NAMES.length; i++) {
                telemetry.addData(Misc.MOTOR_NAMES[i] + " current (A)", robot.misc.currents[i]);
            }
        }
        telemetry.update();
        if(gamepad1.left_stick_button){
            //locked pos
            if(!follower.holding()){
                follower.update();
                follower.hold(follower.pose());
            }
            follower.update();
        }else{
            if(follower.holding()){
                follower.stop();
                robot.dt.brake();
            }
            robot.dt.driveRobot(gamepad1);
        }

        if(gamepad1.backWasReleased()){
            //toggle aiming (turrets + shooters)
            aim = !aim;
        }

        if(gamepad1.left_trigger > 0.7){
            //shoot, so stop blocking and run transfer
            robot.intake.transferBalls();
        }else if(gamepad1.right_trigger > 0.7){
            //run intake
            robot.intake.runIntake();
        }else if(gamepad1.left_bumper){
            //run intake with flower intake down
            robot.intake.runIntake();
        }else if(gamepad1.a){
            //run intake in reverse
            robot.intake.reverseIntake();
        }else{
            robot.intake.stopIntake();
            robot.intake.flowerIntakeUp();
            robot.shooter.block();
        }

        if((gamepad1.left_trigger > 0.7) && aim){
            robot.shooter.release();
        }

        if(gamepad1.left_bumper){
            robot.intake.flowerIntakeDown();
        }


        if(gamepad1.y){
            //TODO: sotm
            //technically sotm but justin can keep dreaming
            sotm = true;
        }else{
            sotm = false;
        }
        if(gamepad1.x){
            alliance = "blue";
        }
        if(gamepad1.b){
            alliance = "red";
        }
        if(gamepad1.dpadLeftWasPressed()){
            robot.dt.relocalize(alliance, true);
        }
        if(gamepad1.dpadRightWasPressed()){
            robot.dt.relocalize(alliance, false);
        }

        if(gamepad1.startWasReleased()){
            //recalibration
            robot.dt.recalibrate();
        }
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
