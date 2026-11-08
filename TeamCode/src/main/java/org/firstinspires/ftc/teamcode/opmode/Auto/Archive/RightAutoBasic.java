package org.firstinspires.ftc.teamcode.opmode.Auto.Archive;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.opmode.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.paths.Path;

@Disabled
@Autonomous
public class RightAutoBasic extends OpMode {

    // Other Constants
    double initialWaitTime = 1000;
    double shootingStartDelay = 500;
    double shootingDelay = 500;
    double flowerDelay = 2500;

    private Robot robot;
    private Intake intake;
    private Shooter shooter;

    // Pedro
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPos = poseFactory.of(53.5, 9, -90);
    private final Pose path1Start = poseFactory.of(53.5, 9, -90);
    private final Pose path1 = poseFactory.of(16, 47.0629, 180);
    private final Pose path1Control1 = poseFactory.of(53.5, 35, 0);
    private final Pose path1Control2 = poseFactory.of(35, 48, 0);
    private final Pose point2 = poseFactory.of(53.2737, 20.851, -142.76);
    private final Pose point2Control1 = poseFactory.of(34.074, 45.8355, 0);
    private final Pose point2Control2 = poseFactory.of(45.5268, 14.5605, 0);
    private final Pose point3 = poseFactory.of(15, 4, -176.4442);
    private final Pose point3Control1 = poseFactory.of(38.5812, 6.6057, 0);
    private final Pose point4 = poseFactory.of(60, 120, -270);
    private final Pose point4Control1 = poseFactory.of(70.8507, 5.9923, 0);
    private final Pose point4Control2 = poseFactory.of(61.915, 24.2427, 0);
    private final Pose point4Segment2Start = poseFactory.of(59.1759, 121.2475, -174.8806);
    private final Pose point4Segment2End = poseFactory.of(59.1759, 121.2475, 90);
    private final Pose point5 = poseFactory.of(48, 125, 94.4625);
    private final Pose point5Control1 = poseFactory.of(47.0482, 106.9387, 0);
    private final Pose point6 = poseFactory.of(15, 111.227, 3.7768);
    private final Pose point6Control1 = poseFactory.of(46.7674, 113.7467, 0);

    public Path flower1() {
        return curve(path1Start, path1Control1, path1Control2, path1).linear(path1Start, path1);
    }

    public Path scoreFlower1() {
        return curve(path1, point2Control1, point2Control2, point2).reverseTangent();
    }

    public Path intakeLine() {
        return curve(point2, point3Control1, point3).tangent();
    }

    public Path scoreLine() {
        return curve(point3, point4Control1, point4Control2, point4).heading(Interpolator.piecewise().until(0.0869, Interpolator.tangent.reverse()).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public Path flower2() {
        return curve(point4, point5Control1, point5).constant(point5);
    }

    public Path park() {
        return curve(point5, point6Control1, point6).tangent();
    }

    private Command autoRoutine() {
        return sequential(
                waitMs(initialWaitTime),
                intake.transferOn(),
                waitMs(shootingDelay),
                parallel(
                        sequential(
                                waitMs(200),
                                intake.closeGate()
                        ),
                        follow(follower, flower1()),
                        sequential(
                                waitMs(200),
                                intake.flowerDown()
                        )
                ),
                waitMs(flowerDelay),
                parallel(
                        follow(follower, scoreFlower1()),
                        sequential(
                                waitMs(200),
                                intake.flowerUp(),
                                waitMs(200),
                                intake.intakeOn()//,
                                //intake.intakeOff()
                        )
                ),
                waitMs(shootingStartDelay),
                intake.transferOn(),
                waitMs(shootingDelay),
                parallel(
                        sequential(
                                waitMs(200),
                                intake.closeGate()
                        ),
                        follow(follower, intakeLine())
                ),
                waitMs(700),
                follow(follower, scoreLine()),
                waitMs(shootingStartDelay),
                intake.transferOn(),
                waitMs(shootingDelay),
                parallel(
                        sequential(
                                waitMs(200),
                                intake.closeGate()
                        ),
                        follow(follower, flower2()),
                        sequential(
                                waitMs(200),
                                intake.flowerDown()
                        )
                ),
                waitMs(flowerDelay),
                intake.transferOn(),
                waitMs(3000),
                follow(follower, park())

        );
    }

    @Override
    public void init() {

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPos);
        follower.update();

        robot = new Robot(hardwareMap);
        robot.alliance = "red"; // TODO: confirm this is the correct alliance for this auto

        intake = new Intake(hardwareMap);
        intake.init();

        shooter = new Shooter(hardwareMap, robot);
        shooter.init();

    }

    @Override
    public void start() {

        schedule(autoRoutine());

    }

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        double x = follower.pose().x();
        double y = follower.pose().y();
        double heading = Math.toDegrees(follower.pose().heading());

        shooter.updateFlywheel(x, y, heading);
        shooter.updateTurret(x, y, heading);

        // Shooter: what the flywheels/hoods are actually using
        double nectarDist = shooter.getNectarTargetDistance(x, y, heading);
        double pollenDist = shooter.getPollenTargetDistance(x, y, heading);
        double nectarTargetVel = shooter.getNectarTargetVelocity(nectarDist);
        double pollenTargetVel = shooter.getPollenTargetVelocity(pollenDist);
        double nectarVel = shooter.getNectarMotorSpeed();
        double pollenVel = shooter.getPollenMotorSpeed();
        double[] turretGoal = shooter.getTurretGoal(y);

        telemetry.addLine("--- Pose ---");
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading", heading);
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Path completion", follower.completion());
        telemetry.addData("Distance remaining in path", follower.remainingDistance());

        telemetry.addLine("--- Flywheels ---");
        telemetry.addData("Nectar velocity / target", "%.0f / %.0f", nectarVel, nectarTargetVel);
        telemetry.addData("Pollen velocity / target", "%.0f / %.0f", pollenVel, pollenTargetVel);
        telemetry.addData("Nectar error", nectarVel - nectarTargetVel);
        telemetry.addData("Pollen error", pollenVel - pollenTargetVel);
        telemetry.addData("Motor power (nectar/pollen)", "%.2f / %.2f", shooter.getNectarMotorPower(), shooter.getPollenMotorPower());

        telemetry.addLine("--- Shot ---");
        telemetry.addData("Turret goal (x, y)", "%.1f, %.1f", turretGoal[0], turretGoal[1]);
        telemetry.addData("Nectar target distance", nectarDist);
        telemetry.addData("Pollen target distance", pollenDist);
        telemetry.addData("Shot zone", shooter.getShotZone());
        telemetry.addData("Hood position (nectar/pollen)", "%.3f / %.3f", shooter.getNectarHoodPosition(), shooter.getPollenHoodPosition());
        telemetry.addData("Shot table error nectar", shooter.getShotTableErrorNectar());
        telemetry.addData("Shot table error pollen", shooter.getShotTableErrorPollen());
        telemetry.addData("Turret servo pos (nectar/pollen)", "%.3f / %.3f", shooter.getNectarTurretServoPosition(), shooter.getPollenTurretServoPosition());
        telemetry.update();

    }
}
