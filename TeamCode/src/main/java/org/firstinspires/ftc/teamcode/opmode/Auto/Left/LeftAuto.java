package org.firstinspires.ftc.teamcode.opmode.Auto.Left;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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

import org.firstinspires.ftc.teamcode.opmode.Auto.AutoConstants;
import org.firstinspires.ftc.teamcode.opmode.Auto.AutoRobot;
import org.firstinspires.ftc.teamcode.opmode.Auto.PathSelector;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.opmode.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;

@Config
@Autonomous
public class LeftAuto extends OpMode {

    // Park side. Pick it during init with gamepad1 or gamepad2 (dpad right = right, dpad left = left).
    private boolean parkRight = true;

    private Robot robot;
    private Intake intake;
    private Shooter shooter;
    private PathSelector pathSelector;
    private AutoRobot autoRobot;

    // Alliance whose AprilTags tip detection looks for. Pick it during init with gamepad1 or gamepad2 (X = blue, B = red).
    private String tipAlliance = "red";

    // Pedro
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose secondary = poseFactory.of(57, 119, 270);

    private final Pose startPos = poseFactory.of(61.5, 137.2, 180);

    private final Pose secondaryToRightPark = poseFactory.of(16, 95, 0);
    private final Pose secondaryToRightParkC1 = poseFactory.of(40, 95, 0);

    private final Pose secondaryToLeftPark = poseFactory.of(16, 120, 0);


    public Path toSecondary() {
        return Paths.line(startPos, secondary).linear(startPos, poseFactory.of(0, 0, 90));
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
                follow(follower, toSecondary()),
                waitMs(AutoConstants.INITIAL_WAIT_TIME),
                pathSelector.shootWhenTipped(),

                // Park
                pathSelector.park(secondaryToRightPark(), secondaryToLeftPark(), parkRight)

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
