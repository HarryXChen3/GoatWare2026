package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.FuelState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.MovingTOFShot;
import frc.robot.subsystems.superstructure.StaticShot;
import frc.robot.subsystems.superstructure.Superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final FuelState fuelState;
    private final Superstructure superstructure;

    public ShootCommands(
            final Swerve swerve,
            final Intake intake,
            final Indexer indexer,
            final FuelState fuelState,
            final Superstructure superstructure
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.indexer = indexer;
        this.fuelState = fuelState;
        this.superstructure = superstructure;
    }

    public Pose2d turretPose(final Pose2d robotPose) {
        return robotPose.plus(superstructure.getOffsetFromCenter());
    }

    public Command intake() {
        return Commands.parallel(
                intake.intake()
        );
    }

    public Command trackHub() {
        return superstructure.runParameters(
                StaticShot.parametersSupplier(
                        swerve::getPose,
                        this::turretPose,
                        swerve::getRobotRelativeSpeeds,
                        FieldConstants::getHubPose
                )
        ).withName("TrackHub");
    }

    public Command stopAndShoot() {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        Commands.parallel(
                                indexer.toFeed()
                                        .onlyWhile(superstructure::atSetpoint),
                                Commands.waitSeconds(2.5)
                                        .andThen(intake.stow())
                        )
                ).onlyWhile(fuelState.hasFuel),
                superstructure.runParameters(
                        StaticShot.parametersSupplier(
                                swerve::getPose,
                                this::turretPose,
                                swerve::getRobotRelativeSpeeds,
                                FieldConstants::getHubPose
                        )
                ),
                swerve.runWheelXCommand()
        ).withName("StopAndShoot");
    }

    public Command shoot() {
        return deadline(
                repeatingSequence(
//                        waitUntil(superstructure::atSetpoint),
                        Commands.parallel(
                                indexer.toFeed(),
//                                        .onlyWhile(superstructure::atSetpoint),
                                Commands.waitSeconds(2.5)
                                        .andThen(intake.stow())
                        )
                ).onlyWhile(fuelState.hasFuel),
                superstructure.runParameters(
                        MovingTOFShot.parametersSupplier(
                                swerve::getPose,
                                this::turretPose,
                                swerve::getRobotRelativeSpeeds,
                                FieldConstants::getHubPose
                        )
                )
        ).withName("Shoot");
    }
}
