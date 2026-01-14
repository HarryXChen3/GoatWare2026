package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.superstructure.StaticShot;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.function.DoubleSupplier;

public class ShootCommands {
    private final Swerve swerve;
    private final Indexer indexer;
    private final Superstructure superstructure;

    public ShootCommands(
            final Swerve swerve,
            final Indexer indexer,
            final Superstructure superstructure
    ) {
        this.swerve = swerve;
        this.indexer = indexer;
        this.superstructure = superstructure;
    }

    public Command stopAndShoot() {
        final DoubleSupplier turretPositionSupplier = () -> {
            final Pose2d robotPose = swerve.getPose();
            final Pose2d hubPose = FieldConstants.getHubPose();

            return hubPose.minus(robotPose).getTranslation().getAngle().getRotations();
        };

        return Commands.deadline(
                Commands.sequence(
                    Commands.idle()
                ),
                superstructure.runParameters(
                        StaticShot.parametersSupplier(swerve::getPose, FieldConstants::getHubPose),
                        turretPositionSupplier
                ).asProxy(),
                swerve.runWheelXCommand()
        );
    }
}
