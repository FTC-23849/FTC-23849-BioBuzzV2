package org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems;

import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

public class Intake {

    public HardwareMap hwmap;
    public DcMotorEx leftIntake, rightIntake;
    public ServoImplEx flowerIntake;
    public ServoImplEx stopper;

    public Intake (HardwareMap hardwareMap) {

        hwmap = hardwareMap;

        leftIntake = hwmap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hwmap.get(DcMotorEx.class, "rightIntake");
        flowerIntake = hwmap.get(ServoImplEx.class, "flowerIntake");
        stopper = hwmap.get(ServoImplEx.class, "stopper");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void init() {

        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);

        flowerIntake.setPosition(flowerIntakeUp);

    }

    void release() {
        stopper.setPosition(releaseBall);
    }

    void block() {
        stopper.setPosition(blockBall);
    }

    public Command intakeOn() {

        return Command.build()
                .setStart(() -> {
                    leftIntake.setPower(intakePower);
                    rightIntake.setPower(intakePower);
                })
                .setDone(() -> true)
                .requiring(leftIntake, rightIntake);

    }

    public Command intakeOff() {

        return Command.build()
                .setStart(() -> {
                    leftIntake.setPower(0.0);
                    rightIntake.setPower(0.0);
                })
                .setDone(() -> true)
                .requiring(leftIntake, rightIntake);

    }

    public Command flowerDown() {

        return Command.build()
                .setStart(() -> {
                    flowerIntake.setPosition(flowerIntakeDown);
                })
                .setDone(() -> true)
                .requiring(flowerIntake);

    }

    public Command flowerUp() {

        return Command.build()
                .setStart(() -> {
                    flowerIntake.setPosition(flowerIntakeUp);
                })
                .setDone(() -> true)
                .requiring(flowerIntake);

    }

    public Command transferOn() {

        return Command.build()
                .setStart(() -> {
                    leftIntake.setPower(transferSpeedAuto);
                    rightIntake.setPower(transferSpeedAuto);
                    release();
                })
                .setDone(() -> true)
                .requiring(leftIntake, rightIntake, stopper);

    }

    public Command transferOff() {

        return Command.build()
                .setStart(() -> {
                    leftIntake.setPower(0.0);
                    rightIntake.setPower(0.0);
                    waitMs(200);
                    block();
                })
                .setDone(() -> true)
                .requiring(leftIntake, rightIntake, stopper);

    }

    public Command openGate() {

        return Command.build()
                .setStart(this::release)
                .setDone(() -> true)
                .requiring(stopper);

    }

    public Command closeGate() {

        return Command.build()
                .setStart(this::block)
                .setDone(() -> true)
                .requiring(stopper);

    }

}
