package org.firstinspires.ftc.teamcode.opmode.Auto.Right;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Auto.AutoConstants;
import org.firstinspires.ftc.teamcode.opmode.Auto.AutoRobot;
import org.firstinspires.ftc.teamcode.opmode.Auto.PathSelector;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.opmode.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;

@Config
@Autonomous
public class RightAuto extends OpMode {

    // Park side. Pick it during init with gamepad1 or gamepad2 (dpad right = right, dpad left = left).
    private boolean parkRight = true;

    private Robot robot;
    private Intake intake;
    private Shooter shooter;
    private PathSelector pathSelector;
    private AutoRobot autoRobot;

    // Alliance whose AprilTags tip detection looks for. Pick it during init with gamepad1 or gamepad2 (X = blue, B = red).
    private String tipAlliance = "red";

    // Timeout park: when PARK_TIME_LEFT_SECONDS are left, drop the routine and park
    private final ElapsedTime autoTimer = new ElapsedTime();
    private Command routine;
    private boolean parking = false;

    // Pedro
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose primary = poseFactory.of(57, 23, -90);
    private final Pose secondary = poseFactory.of(57, 119, 270);

    private final Pose startPos = poseFactory.of(61.5, 6.8, 180);
    private final Pose intakeLine = poseFactory.of(11, 6.8, 180);

    private final Pose lineToPrimaryControl1 = poseFactory.of(57, 6.8, 0);
    private final Pose primaryToSecondaryReverseControl1 = poseFactory.of(61, 72, 0);

    private final Pose flower1 = poseFactory.of(14, 48, 180);
    private final Pose flower2 = poseFactory.of(48, 130, 90);

    private final Pose primaryToFlower1C1 = poseFactory.of(40, 23, 0);
    private final Pose primaryToFlower1C2 = poseFactory.of(32, 48, 0);

    private final Pose primaryToFlower2C1 = poseFactory.of(70, 85, 0);
    private final Pose primaryToFlower2C2 = poseFactory.of(48, 110, 0);

    private final Pose secondaryToFlower1C1 = poseFactory.of(16, 119, 0);
    private final Pose secondaryToFlower1C2 = poseFactory.of(56, 46, 0);

    private final Pose secondaryToFlower2C1 = poseFactory.of(48, 100, 0);

    private final Pose flower1ToSecondaryC1 = poseFactory.of(50, 46, 0);
    private final Pose flower1ToSecondaryC2 = poseFactory.of(10, 119, 0);

    private final Pose flower2ToPrimaryC1 = poseFactory.of(65, 100, 0);
    private final Pose flower2ToSecondaryC1 = poseFactory.of(48, 100, 0);

    private final Pose primaryToRightPark = poseFactory.of(14, 95, 0);
    private final Pose primaryToRightParkC1 = poseFactory.of(24, 42, 0);

    private final Pose primaryToLeftPark = poseFactory.of(19, 120, 0);
    private final Pose primaryToLeftParkC1 = poseFactory.of(11, 34, 0);
    private final Pose primaryToLeftParkC2 = poseFactory.of(50, 120, 0);

    private final Pose secondaryToRightPark = poseFactory.of(16, 95, 0);
    private final Pose secondaryToRightParkC1 = poseFactory.of(40, 95, 0);

    private final Pose secondaryToLeftPark = poseFactory.of(16, 120, 0);


    public Path intakeLine() {
        return Paths.line(startPos, intakeLine).constant(intakeLine);
    }

    public Path lineToPrimary() {
        return Paths.curve(intakeLine, lineToPrimaryControl1, primary).reverseTangent();
    }

    public Path primaryToSecondary() {
        return Paths.curve(primary, primaryToSecondaryReverseControl1, secondary).constant(poseFactory.of(0, 0, 290));
    }

    public Path secondaryToPrimary() {
        return Paths.curve(secondary, primaryToSecondaryReverseControl1, primary).constant(poseFactory.of(0, 0, 90));
    }

    public Path primaryToFlower1() {
        return Paths.curve(primary, primaryToFlower1C1, primaryToFlower1C2, flower1).tangent();
    }

    public Path secondaryToFlower1() {
        return Paths.curve(secondary, secondaryToFlower1C1, secondaryToFlower1C2, flower1).tangent();
    }

    public Path flower1ToPrimary() {
        return Paths.line(flower1, primary).linear(flower1, poseFactory.of(0, 0, 270));
    }

    public Path flower1ToSecondary() {
        return Paths.curve(flower1, flower1ToSecondaryC1, flower1ToSecondaryC2, secondary).linear(flower1, poseFactory.of(0, 0, 90));
    }

    public Path primaryToFlower2() {
        return Paths.curve(primary, primaryToFlower2C1, primaryToFlower2C2, flower2).tangent();
    }

    public Path secondaryToFlower2() {
        return Paths.curve(secondary, secondaryToFlower2C1, flower2).constant(poseFactory.of(0, 0, 90));
    }

    public Path flower2ToPrimary() {
        return Paths.curve(flower2, flower2ToPrimaryC1, primary).constant(poseFactory.of(0, 0, 270));
    }

    public Path flower2ToSecondary() {
        return Paths.curve(flower2, flower2ToSecondaryC1, secondary).constant(poseFactory.of(0, 0, 90));
    }

    public Path primaryToRightPark() {
        return Paths.curve(primary, primaryToRightParkC1, primaryToRightPark).tangent();
    }

    public Path primaryToLeftPark() {
        return Paths.curve(primary, primaryToLeftParkC1, primaryToLeftParkC2, primaryToLeftPark).linear(primary, primaryToLeftPark);
    }

    public Path secondaryToRightPark() {
        return Paths.curve(secondary, secondaryToRightParkC1, secondaryToRightPark).constant(secondaryToRightPark);
    }

    public Path secondaryToLeftPark() {
        return Paths.line(secondary, secondaryToLeftPark).constant(secondaryToLeftPark);
    }


    private Command autoRoutine() {
        return sequential(

                // Preloads
                waitMs(AutoConstants.INITIAL_WAIT_TIME),
                autoRobot.shoot(),

                // Line
                autoRobot.followAndIntake(intakeLine()),
                waitMs(AutoConstants.INTAKE_WAIT_TIME), // Let balls intake
                pathSelector.toShootingSpot(lineToPrimary()),
                pathSelector.shootAtBestSpot(primaryToSecondary()),

                // Flower 1
                pathSelector.toPickup(primaryToFlower1(), secondaryToFlower1(), true),
                waitMs(AutoConstants.FLOWER_WAIT_TIME), // Let balls intake
                pathSelector.toShootingSpot(flower1ToPrimary(), flower1ToSecondary()),
                pathSelector.shootAtBestSpot(primaryToSecondary(), secondaryToPrimary()),

                // Flower 2
                pathSelector.toPickup(primaryToFlower2(), secondaryToFlower2(), true),
                waitMs(AutoConstants.FLOWER_WAIT_TIME), // Let balls intake
                pathSelector.toShootingSpot(flower2ToPrimary(), flower2ToSecondary()),
                pathSelector.shootAtBestSpot(primaryToSecondary(), secondaryToPrimary()),

                // Park
                pathSelector.park(primaryToRightPark(), primaryToLeftPark(), secondaryToRightPark(), secondaryToLeftPark(), parkRight)

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

        autoRobot = new AutoRobot(shooter, intake, follower);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void init_loop() {

        if (gamepad1.x || gamepad2.x) {
            tipAlliance = "blue";
        } else if (gamepad1.b || gamepad2.b) {
            tipAlliance = "red";
        }

        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            parkRight = true;
        } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            parkRight = false;
        }

        telemetry.addData("Tip detection alliance", tipAlliance);
        telemetry.addLine("gamepad1/2: X = blue, B = red");
        telemetry.addData("Park side", parkRight ? "right" : "left");
        telemetry.addLine("gamepad1/2: dpad right = right, dpad left = left");
        telemetry.update();

    }

    @Override
    public void start() {

        // Built here (not in init) so it gets the alliance picked during init_loop
        pathSelector = new PathSelector(follower, shooter, intake, robot, autoRobot, tipAlliance);

        routine = autoRoutine();
        schedule(routine);
        autoTimer.reset();

    }

    @Override
    public void loop() {

        follower.update();

        // Running out of time, the routine hasn't finished, and it isn't already parking: drop it and park
        if (!parking && Scheduler.isScheduled(routine) && !pathSelector.hasStartedPark()
                && autoTimer.seconds() >= AutoConstants.AUTO_DURATION_SECONDS - AutoConstants.PARK_TIME_LEFT_SECONDS) {
            parking = true;
            Scheduler.reset(); // clears the routine (the new park path overrides the current one)
            schedule(pathSelector.timeoutPark(primary, secondary,
                    primaryToRightPark(), primaryToLeftPark(),
                    secondaryToRightPark(), secondaryToLeftPark(),
                    parkRight));
        }

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
