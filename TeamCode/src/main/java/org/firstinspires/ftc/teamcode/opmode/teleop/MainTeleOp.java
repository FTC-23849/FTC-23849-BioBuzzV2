package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.powerAdjustmentStep;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.turretAdjustmentStep;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.turretBackCenterPosNectar;

import org.firstinspires.ftc.teamcode.opmode.subsystems.Misc;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;
//testing commit
@Config
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;
    public static double turretPositionTesting = turretBackCenterPosNectar;
    public static double flywheelSpeedTesting = 1200;
    public static double hoodPositionTesting = 0.25;
    public static boolean showServoStates = false;
    public static boolean showCurrentDraw = false;
    boolean testingShooter = false;
    boolean testingTurret = false;
    public static double turretAdjustment = 0;
    public static double powerAdjustment = 0;
    boolean aim = false;
    boolean sotm = false;
    String alliance = "red";
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
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
        robot.misc.limelight.stop();
        robot.misc.startLogging();
    }

    @Override
    public void loop() {
        if(gamepad2.leftBumperWasReleased()){
            //toggle testing, which switches from dynamic aiming and power to static aiming and power,
            // for testing and if the robot break
            testingShooter = !testingShooter;
        }
        if(gamepad2.rightBumperWasReleased()){
            testingTurret = !testingTurret;
        }
        if(gamepad2.startWasReleased()){
            turretAdjustment += turretAdjustmentStep;
        }
        if(gamepad2.backWasReleased()){
            turretAdjustment -= turretAdjustmentStep;
        }
        if(gamepad2.dpadUpWasReleased()){
            powerAdjustment += powerAdjustmentStep;
        }
        if(gamepad2.dpadDownWasReleased()){
            powerAdjustment -= powerAdjustmentStep;
        }
        robot.turretAdjustment = turretAdjustment;
        robot.powerAdjustment = powerAdjustment;
        robot.testingShooter = testingShooter;
        robot.testingTurret = testingTurret;
        robot.hoodPositionTesting = hoodPositionTesting;
        robot.turretPositionTesting = turretPositionTesting;
        robot.flywheelSpeedTesting = flywheelSpeedTesting;
        robot.misc.showCurrentDraw = showCurrentDraw;
        robot.update(aim, sotm, alliance);
        robot.addTelemetry();
        telemetry.addData("cfg flywheelSpeedTesting", flywheelSpeedTesting);
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
        robot.dt.driveRobot(gamepad1);

        if(gamepad1.backWasReleased()){
            //toggle aiming (turret + shooter)
            aim = !aim;
        }

        if(gamepad1.left_trigger > 0.7){
            //shoot, so stop blocking and run transfer
            robot.intake.transferBalls();
        }else if(gamepad1.right_trigger > 0.7){
            //run intake
            robot.intake.runIntake();
        }else if(gamepad1.left_bumper){
            //run intake with flower intake down (flower lowered below)
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
        //TODO: additional buttons like locked button / button remapping
        //tbd controls, button for locked pos?
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
