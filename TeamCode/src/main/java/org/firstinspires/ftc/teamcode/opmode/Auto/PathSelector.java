package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;

import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

public class PathSelector {

    private final Follower follower;
    private final Shooter shooter;
    private final Intake intake;
    private final Robot robot;
    private final AutoRobot autoRobot;

    // Which alliance's AprilTags canShoot looks for ("red" or "blue").
    // Separate from robot.alliance so a red-written auto can run on blue.
    private final String alliance;

    // Which spot we're currently using. Sticks until that spot stops working.
    private boolean atPrimarySpot = true;

    // Set as soon as any park() starts, so the auto knows not to start a timeout park
    private boolean parkStarted = false;

    public PathSelector(Follower follower, Shooter shooter, Intake intake, Robot robot, AutoRobot autoRobot, String alliance) {
        this.follower = follower;
        this.shooter = shooter;
        this.intake = intake;
        this.robot = robot;
        this.autoRobot = autoRobot;
        this.alliance = alliance;
    }

    // Drive from a pickup back to whichever spot we're currently using
    // (raises the flower on the way if it's down)
    public Command toShootingSpot(Path toPrimary, Path toSecondary) {
        return lazy(() -> autoRobot.followLeavingFlower(atPrimarySpot ? toPrimary : toSecondary));
    }

    // Only one path from this pickup, and it ends at the primary spot
    public Command toShootingSpot(Path toPrimary) {
        return toShootingSpot(toPrimary, true);
    }

    // Only one path from this pickup; say which spot it ends at
    // (true = primary, false = secondary) so the flag stays correct
    public Command toShootingSpot(Path path, boolean endsAtPrimary) {
        return lazy(() -> {
            atPrimarySpot = endsAtPrimary;
            return autoRobot.followLeavingFlower(path);
        });
    }

    // Shoot from the current spot if the camera can see the target,
    // otherwise drive to the other spot (and stay on that one from now on) and shoot there
    public Command shootAtBestSpot(Path primaryToSecondary, Path secondaryToPrimary) {
        return sequential(
                waitMs(AutoConstants.TIP_DETECTION_WAIT_TIME),
                lazy(() -> {
                    if (shooter.canShoot(alliance)) {
                        return waitMs(0);
                    }
                    Path switchPath = atPrimarySpot ? primaryToSecondary : secondaryToPrimary;
                    atPrimarySpot = !atPrimarySpot;
                    return follow(follower, switchPath);
                }),
                autoRobot.shoot()
        );
    }

    // Only one switch path, going primary -> secondary
    public Command shootAtBestSpot(Path primaryToSecondary) {
        return shootAtBestSpot(primaryToSecondary, true);
    }

    // Only one switch path; say which spot it starts from (true = primary, false = secondary).
    // If the camera can't see the target but the robot is at the OTHER spot (no path from there),
    // it stays put and shoots anyway.
    public Command shootAtBestSpot(Path switchPath, boolean startsAtPrimary) {
        return sequential(
                waitMs(AutoConstants.TIP_DETECTION_WAIT_TIME),
                lazy(() -> {
                    if (shooter.canShoot(alliance) || atPrimarySpot != startsAtPrimary) {
                        return waitMs(0);
                    }
                    atPrimarySpot = !atPrimarySpot;
                    return follow(follower, switchPath);
                }),
                autoRobot.shoot()
        );
    }

    // Drive to the next pickup from whichever spot we actually shot at
    public Command toPickup(Path fromPrimary, Path fromSecondary) {
        return lazy(() -> follow(follower, atPrimarySpot ? fromPrimary : fromSecondary));
    }

    // Drive to the next pickup and get the mechanism ready on the way:
    // toFlower = true  -> flower down (followAndFlower)
    // toFlower = false -> intake on (followAndIntake)
    public Command toPickup(Path fromPrimary, Path fromSecondary, boolean toFlower) {
        return lazy(() -> {
            Path path = atPrimarySpot ? fromPrimary : fromSecondary;
            return toFlower ? autoRobot.followAndFlower(path) : autoRobot.followAndIntake(path);
        });
    }

    // Same, but wait mechanismDelay ms after the path starts before dropping the flower/intake
    public Command toPickup(Path fromPrimary, Path fromSecondary, boolean toFlower, double mechanismDelay) {
        return lazy(() -> {
            Path path = atPrimarySpot ? fromPrimary : fromSecondary;
            return toFlower
                    ? autoRobot.followAndFlower(path, mechanismDelay)
                    : autoRobot.followAndIntake(path, mechanismDelay);
        });
    }

    // Park from whichever spot we're at. parkRight = true -> right park, false -> left park.
    // Also turns the intake off and raises the flower intake.
    public Command park(Path primaryToRight, Path primaryToLeft,
                        Path secondaryToRight, Path secondaryToLeft,
                        boolean parkRight) {
        return lazy(() -> {
            parkStarted = true;
            Path path;
            if (atPrimarySpot) {
                path = parkRight ? primaryToRight : primaryToLeft;
            } else {
                path = parkRight ? secondaryToRight : secondaryToLeft;
            }
            return parallel(
                    follow(follower, path),
                    intake.intakeOff(),
                    intake.flowerUp()
            );
        });
    }

    // Park when you know the robot is only ever coming from one place:
    // just give the right and left park paths from there.
    // Also turns the intake off and raises the flower intake.
    public Command park(Path toRight, Path toLeft, boolean parkRight) {
        return lazy(() -> {
            parkStarted = true;
            return parallel(
                    follow(follower, parkRight ? toRight : toLeft),
                    intake.intakeOff(),
                    intake.flowerUp()
            );
        });
    }

    // Out of time: drive straight from wherever the robot is to the closer shooting spot,
    // then park from there. Intake goes off and the flower comes up right away so nothing
    // is hanging down while it drives.
    public Command timeoutPark(Pose primary, Pose secondary,
                               Path primaryToRight, Path primaryToLeft,
                               Path secondaryToRight, Path secondaryToLeft,
                               boolean parkRight) {
        return lazy(() -> {
            Pose here = follower.pose();
            double toPrimary = Math.hypot(here.x() - primary.x(), here.y() - primary.y());
            double toSecondary = Math.hypot(here.x() - secondary.x(), here.y() - secondary.y());

            boolean primaryCloser = toPrimary <= toSecondary;
            Pose spot = primaryCloser ? primary : secondary;
            atPrimarySpot = primaryCloser; // so park() picks the paths that start at this spot

            // Already basically at the spot: skip the zero-length drive
            Command toSpot = Math.min(toPrimary, toSecondary) < 1
                    ? waitMs(0)
                    : follow(follower, Paths.line(here, spot).constant(spot));

            return sequential(
                    parallel(
                            toSpot,
                            intake.intakeOff(),
                            intake.flowerUp()
                    ),
                    park(primaryToRight, primaryToLeft, secondaryToRight, secondaryToLeft, parkRight)
            );
        });
    }

    // Wait until the camera sees our alliance's tags (hive tipped so we can shoot),
    // wait AutoConstants.TIP_DELAY ms more for it to settle, then shoot
    public Command shootWhenTipped() {
        return sequential(
                waitUntil(() -> shooter.canShoot(alliance)),
                waitMs(AutoConstants.TIP_DELAY),
                autoRobot.shoot()
        );
    }

    // True once a park() has started (used to skip the timeout park)
    public boolean hasStartedPark() {
        return parkStarted;
    }

    // For telemetry
    public boolean isAtPrimarySpot() {
        return atPrimarySpot;
    }
}
