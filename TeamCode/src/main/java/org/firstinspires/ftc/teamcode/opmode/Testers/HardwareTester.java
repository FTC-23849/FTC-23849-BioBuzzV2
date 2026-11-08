package org.firstinspires.ftc.teamcode.opmode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Drivetrain;

import java.util.List;

// Controls (gamepad1):
//   dpad up/down     select device
//   motor selected:  left stick y = power (only while held), X = reset encoder
//   servo selected:  dpad left/right = change target, bumpers = change step size,
//                    A = enable servo + send target (then edits apply live), B = disable servo (goes limp)
// Servos are never commanded until you press A, so nothing moves on init/start.
@TeleOp(name = "Hardware Tester", group = "Test")
public class HardwareTester extends OpMode {
    static final String[] MOTOR_NAMES = {"LF", "RF", "LB", "RB", "leftIntake", "rightIntake", "nectarShooter", "pollenShooter"};
    static final String[] SERVO_NAMES = {"flowerIntake", "stopper",
            "nectarHood", "frontTurretNectar", "backTurretNectar",
            "pollenHood", "frontTurretPollen", "backTurretPollen"};
    static final double[] STEP_SIZES = {0.1, 0.01, 0.001};

    Drivetrain dt;
    List<LynxModule> hubs;
    DcMotorEx[] motors = new DcMotorEx[MOTOR_NAMES.length];
    ServoImplEx[] servos = new ServoImplEx[SERVO_NAMES.length];
    double[] servoTargets = new double[SERVO_NAMES.length];
    boolean[] servoEnabled = new boolean[SERVO_NAMES.length];
    int selected = 0;
    int stepIndex = 1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //uses Drivetrain so motor directions and pinpoint setup match the real robot
        dt = new Drivetrain(hardwareMap);
        dt.init();

        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
        }
        for (int i = 0; i < SERVO_NAMES.length; i++) {
            servos[i] = hardwareMap.get(ServoImplEx.class, SERVO_NAMES[i]);
            //last commanded position, not the real position; just a starting point for the target
            double lastPos = servos[i].getPosition();
            if (Double.isNaN(lastPos)) {
                servoTargets[i] = 0.5;
            } else {
                servoTargets[i] = lastPos;
            }
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        dt.update(false);

        int deviceCount = MOTOR_NAMES.length + SERVO_NAMES.length;
        if (gamepad1.dpadUpWasPressed()) {
            selected = (selected - 1 + deviceCount) % deviceCount;
        }
        if (gamepad1.dpadDownWasPressed()) {
            selected = (selected + 1) % deviceCount;
        }

        //motors: only the selected one runs, and only while the stick is held
        for (int i = 0; i < motors.length; i++) {
            if (i == selected) {
                motors[i].setPower(-gamepad1.left_stick_y);
            } else {
                motors[i].setPower(0);
            }
        }
        if (selected < motors.length && gamepad1.xWasPressed()) {
            DcMotor.RunMode mode = motors[selected].getMode();
            motors[selected].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[selected].setMode(mode);
        }

        //servos
        if (selected >= motors.length) {
            int s = selected - motors.length;
            if (gamepad1.rightBumperWasPressed()) {
                stepIndex = Math.min(stepIndex + 1, STEP_SIZES.length - 1);
            }
            if (gamepad1.leftBumperWasPressed()) {
                stepIndex = Math.max(stepIndex - 1, 0);
            }
            if (gamepad1.dpadRightWasPressed()) {
                servoTargets[s] = Math.min(servoTargets[s] + STEP_SIZES[stepIndex], 1);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                servoTargets[s] = Math.max(servoTargets[s] - STEP_SIZES[stepIndex], 0);
            }
            if (gamepad1.aWasPressed()) {
                //set position first so enabling never jumps to a stale position
                servos[s].setPosition(servoTargets[s]);
                servos[s].setPwmEnable();
                servoEnabled[s] = true;
            }
            if (gamepad1.bWasPressed()) {
                servos[s].setPwmDisable();
                servoEnabled[s] = false;
            }
            if (servoEnabled[s]) {
                servos[s].setPosition(servoTargets[s]);
            }
        }

        telemetry.addLine("dpad up/down: select | motor: left stick = power, X = reset encoder");
        telemetry.addLine("servo: dpad L/R = target, bumpers = step, A = enable/send, B = limp");
        telemetry.addData("Servo step", STEP_SIZES[stepIndex]);
        telemetry.addLine();

        telemetry.addLine("--- Motors (encoder ticks, ticks/s, power) ---");
        for (int i = 0; i < motors.length; i++) {
            String marker = " ";
            if (i == selected) {
                marker = ">";
            }
            telemetry.addLine(String.format("%s %s: pos %d | vel %.0f | pwr %.2f",
                    marker, MOTOR_NAMES[i],
                    motors[i].getCurrentPosition(), motors[i].getVelocity(), motors[i].getPower()));
        }

        telemetry.addLine("--- Servos (target, state) ---");
        for (int i = 0; i < servos.length; i++) {
            String marker = " ";
            if (i + motors.length == selected) {
                marker = ">";
            }
            String state = "off";
            if (servoEnabled[i]) {
                state = "ON";
            }
            telemetry.addLine(String.format("%s %s: %.3f | %s",
                    marker, SERVO_NAMES[i],
                    servoTargets[i], state));
        }

        Pose2D pos = dt.getRobotPos();
        telemetry.addLine("--- Pinpoint ---");
        telemetry.addData("x (in)", pos.getX(DistanceUnit.INCH));
        telemetry.addData("y (in)", pos.getY(DistanceUnit.INCH));
        telemetry.addData("heading (deg)", dt.getRobotAngle());
        telemetry.addData("raw encoder x", dt.pinpoint.getEncoderX());
        telemetry.addData("raw encoder y", dt.pinpoint.getEncoderY());
        telemetry.update();
    }
}
