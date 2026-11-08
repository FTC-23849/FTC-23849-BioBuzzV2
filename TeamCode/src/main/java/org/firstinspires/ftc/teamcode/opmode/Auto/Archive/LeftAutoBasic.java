package org.firstinspires.ftc.teamcode.opmode.Auto.Archive;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
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
public class LeftAutoBasic extends OpMode {

    // Other Constants
    double initialWaitTime = 10000;
    double shootingStartDelay = 500;
    double shootingDelay = 500;
    double flowerDelay = 3000;

    private Robot robot;
    private Intake intake;
    private Shooter shooter;

    // Pedro
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPos = poseFactory.of(64.5, 132, 90);
    private final Pose flower1 = poseFactory.of(48, 128, 90);
    private final Pose flower1Control1 = poseFactory.of(64.5, 120, 0);
    private final Pose flower1Control2 = poseFactory.of(48, 100, 0);
    private final Pose reverseFlower1 = poseFactory.of(58, 130, 179.7409);
    private final Pose reverseFlower1Control1 = poseFactory.of(48, 115, 0);
    private final Pose flower2 = poseFactory.of(12, 50, -178.066);
    private final Pose flower2Control1 = poseFactory.of(25.6679, 115, 0);
    private final Pose flower2Control2 = poseFactory.of(40, 48, 0);
    private final Pose scoreFlower2 = poseFactory.of(58, 130, -178.7932);
    private final Pose scoreFlower2Control1 = poseFactory.of(35, 48, 0);
    private final Pose scoreFlower2Control2 = poseFactory.of(26, 130, 0);
    private final Pose park = poseFactory.of(15, 115, 0);

    public Path flower1() {
        return curve(startPos, flower1Control1, flower1Control2, flower1).constant(flower1);
    }

    public Path reverseFlower1() {
        return curve(flower1, reverseFlower1Control1, reverseFlower1).reverseTangent();
    }

    public Path flower2() {
        return curve(reverseFlower1, flower2Control1, flower2Control2, flower2).tangent();
    }

    public Path scoreFlower2() {
        return curve(flower2, scoreFlower2Control1, scoreFlower2Control2, scoreFlower2).reverseTangent();
    }

    public Path park() {
        return line(scoreFlower2, park);
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
                        follow(follower, reverseFlower1()),
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
                        follow(follower, flower2()),
                        sequential(
                                waitMs(200),
                                intake.flowerDown()
                        )
                ),
                waitMs(flowerDelay),
                parallel(
                        follow(follower, scoreFlower2()),
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
