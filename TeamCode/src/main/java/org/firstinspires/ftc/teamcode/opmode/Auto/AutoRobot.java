package org.firstinspires.ftc.teamcode.opmode.Auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.flowerIntakeDown;
import com.pedropathing.paths.Path;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Shooter;

public class AutoRobot {

    private final Shooter shooter;
    private final Intake intake;
    private final Follower follower;

    public AutoRobot(Shooter shooter, Intake intake, Follower follower) {

        this.shooter = shooter;
        this.intake = intake;
        this.follower = follower;

    }

    public Command shoot() {

        return sequential(
                waitMs(AutoConstants.SHOOTING_START_DELAY),
                intake.transferOn(),
                waitMs(AutoConstants.SHOOTING_DELAY),
                intake.closeGate()
        );

    }

    public Command leaveFlower() {

        return sequential(
                waitMs(AutoConstants.LEAVE_FLOWER_UP_DELAY),
                intake.flowerUp(),
                waitMs(200),
                intake.intakeOn()
        );

    }

    public Command followAndIntake(Path path) {

        return parallel(
                follow(follower, path),
                intake.intakeOn()
        );

    }

    public Command followAndFlower(Path path) {

        return parallel(
                follow(follower, path),
                intake.flowerDown()
        );

    }

    // Same as followAndIntake, but turns the intake on delay ms after the path starts
    public Command followAndIntake(Path path, double delay) {

        return parallel(
                follow(follower, path),
                sequential(
                        waitMs(delay),
                        intake.intakeOn()
                )
        );

    }

    // Same as followAndFlower, but drops the flower delay ms after the path starts
    public Command followAndFlower(Path path, double delay) {

        return parallel(
                follow(follower, path),
                sequential(
                        waitMs(delay),
                        intake.flowerDown()
                )
        );

    }

    // Follow the path; if the flower intake is down, raise it (leaveFlower) on the way
    public Command followLeavingFlower(Path path) {

        if (intake.flowerIntake.getPosition() == flowerIntakeDown) {
            return parallel(
                    follow(follower, path),
                    leaveFlower()
            );
        }
        return follow(follower, path);

    }

}
