package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.FuelState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.MovingTOFShot;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.subsystems.superstructure.StaticShot;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

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

    public Command intake() {
        return Commands.parallel(
                intake.intake()
        );
    }

    public Command trackHub() {
        final Supplier<ShotParameters> staticParameterSupplier = StaticShot.parametersSupplier(
                swerve::getPose,
                superstructure::getTurretTranslation,
                swerve::getRobotRelativeSpeeds,
                FieldConstants::getHubPose
        );
        final Supplier<ShotParameters> movingTOFParametersSupplier = MovingTOFShot.parametersSupplier(
                swerve::getPose,
                superstructure::getTurretTranslation,
                () -> {
                    final SwerveSpeed.Speeds speedSetpoint = SwerveSpeed.Speeds.SHOOT_AND_SCOOT;
                    final ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();

                    final double linearSpeed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
                    final ChassisSpeeds limitedSpeeds = robotSpeeds
                            .div(linearSpeed)
                            .times(Math.min(linearSpeed, speedSetpoint.getTranslationSpeed()));

                    Logger.recordOutput("Limited", limitedSpeeds);
                    return limitedSpeeds;
                },
                FieldConstants::getHubPose
        );

        return superstructure.runParameters(
                () -> {
                    final ChassisSpeeds fieldSpeeds = swerve.getFieldRelativeSpeeds();
                    final double linearSpeedMetersPerSec = Math.hypot(
                            fieldSpeeds.vxMetersPerSecond,
                            fieldSpeeds.vyMetersPerSecond
                    );

                    return linearSpeedMetersPerSec <= 1e-3
                            ? staticParameterSupplier.get()
                            : movingTOFParametersSupplier.get();
                }
        ).withName("TrackHub");
    }

    public Command stopAndShoot() {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        Commands.deadline(
                                indexer.toFeed()
                                        .onlyWhile(superstructure::atSetpoint),
                                intake.stowFeed()
                        )
                ).onlyWhile(fuelState.hasFuel),
                superstructure.runParameters(
                        StaticShot.parametersSupplier(
                                swerve::getPose,
                                superstructure::getTurretTranslation,
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
                        waitUntil(superstructure::atSetpoint),
                        Commands.deadline(
                                indexer.toFeed()
                                        .onlyWhile(superstructure::atSetpoint),
                                intake.stowFeed()
                        )
                )
                        .onlyIf(fuelState.hasFuel)
                        .onlyWhile(fuelState.hasFuel),
                Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SHOOT_AND_SCOOT),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ),
                superstructure.runParameters(
                        MovingTOFShot.parametersSupplier(
                                swerve::getPose,
                                superstructure::getTurretTranslation,
                                swerve::getRobotRelativeSpeeds,
                                FieldConstants::getHubPose
                        )
                )
        ).withName("Shoot");
    }
}
