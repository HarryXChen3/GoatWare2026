package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexers.Indexer;
import frc.robot.subsystems.superstructure.StaticShot;
import frc.robot.subsystems.superstructure.Superstructure;

@SuppressWarnings("ClassCanBeRecord")
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
        return Commands.deadline(
                Commands.repeatingSequence(
                        Commands.waitUntil(superstructure::atSetpoint),
                        indexer.runFeed()
                                .onlyWhile(superstructure::atSetpoint)
                ).onlyWhile(indexer.hasFuel),
                superstructure.toParameters(
                        StaticShot.parametersSupplier(swerve::getPose, FieldConstants::getHubPose),
                        () -> {
                            final Pose2d hubPose = FieldConstants.getHubPose();
                            final Pose2d robotPose = swerve.getPose();

                            final Rotation2d angle = hubPose.getTranslation()
                                    .minus(robotPose.getTranslation())
                                    .getAngle()
                                    .minus(robotPose.getRotation());

                            return angle.getRotations();
                        }
                ),
                swerve.runWheelXCommand()
        );
    }
}
