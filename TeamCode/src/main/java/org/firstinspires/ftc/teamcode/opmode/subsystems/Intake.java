package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

public class Intake {
    public HardwareMap hwmap;
    public DcMotorEx leftIntake, rightIntake;
    public ServoImplEx flowerIntake;
    public boolean running = false;
    public boolean outtaking = false;
    public Intake(HardwareMap hwmap){
        this.hwmap = hwmap;
    }

    public void transferBalls(){
        running = true;
        outtaking = false;
        leftIntake.setPower(transferSpeed);
        rightIntake.setPower(transferSpeed);
    }
    public void reverseIntake(){
        running = true;
        outtaking = true;
        leftIntake.setPower(reverseIntakePower);
        rightIntake.setPower(reverseIntakePower);
    }

    public void stopIntake(){
        running = false;
        outtaking = false;
        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);
    }

    public void runIntake(){
        running = true;
        outtaking = false;
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
    }

    public void flowerIntakeDown(){
        flowerIntake.setPosition(flowerIntakeDown);
    }

    public void flowerIntakeUp(){
        flowerIntake.setPosition(flowerIntakeUp);
    }

    public void init(){
        leftIntake = hwmap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hwmap.get(DcMotorEx.class, "rightIntake");
        flowerIntake = hwmap.get(ServoImplEx.class, "flowerIntake");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    public void update(){

    }
}
