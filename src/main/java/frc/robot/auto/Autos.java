package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.ShootCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.FuelState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.MovingTOFShot;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.subsystems.superstructure.StaticShot;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.commands.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Autos {
    public static final String LogKey = "Auto";
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final FuelState fuelState;
    private final Superstructure superstructure;

    private final AutoFactory autoFactory;

    private final Supplier<ShotParameters> staticShot;
    private final Supplier<ShotParameters> movingTOFShot;

    private final LoggedTrigger robotStopped;
    private final LoggedTrigger targetIsHub;
    private final LoggedTrigger turretSafe;

    public Autos(
            final Swerve swerve,
            final Intake intake,
            final Indexer indexer,
            final FuelState fuelState,
            final Superstructure superstructure,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.indexer = indexer;
        this.fuelState = fuelState;
        this.superstructure = superstructure;
        this.autoFactory = new AutoFactory(
                swerve::getPose,
                photonVision::resetPose,
                swerve::followChoreoSample,
                true,
                swerve,
                (trajectory, trajectoryRunning) -> {
                    Logger.recordOutput(
                            LogKey + "/Trajectory/Path",
                            (Robot.IsRedAlliance.getAsBoolean() ? trajectory.flipped() : trajectory).getPoses()
                    );

                    Logger.recordOutput(
                            LogKey + "/Trajectory/Name",
                            trajectory.name()
                    );

                    Logger.recordOutput(
                            LogKey + "/Trajectory/Running",
                            trajectoryRunning
                    );
                }
        );

        this.staticShot = staticParameters(swerve::getPose);
        this.movingTOFShot = MovingTOFShot.parametersSupplier(
                swerve::getPose,
                superstructure::getTurretTranslation,
                swerve::getRobotRelativeSpeeds,
                FieldConstants::getHubPose
        );

        this.robotStopped = group.t("RobotStopped",
                () -> ShootCommands.linearSpeed(swerve.getFieldRelativeSpeeds()) <= 0.01
        );
        this.targetIsHub = group.t("TargetIsHub",
                () -> ShootCommands.getTarget(swerve.getPose()) == ShootCommands.Target.HUB
        );
        this.turretSafe = group.t("TurretSafe",
                () -> {
                    final double safeXClose = FieldConstants.getTurretSafeXCloseBoundary();
                    final double safeXFar = FieldConstants.getTurretSafeXFarBoundary();
                    final double turretX = superstructure
                            .getTurretTranslation(swerve.getPose())
                            .getX();
                    return Robot.IsRedAlliance.getAsBoolean()
                            ? (turretX >= safeXClose || turretX <= safeXFar)
                            : (turretX <= safeXClose || turretX >= safeXFar);
                }
        );
    }

    private Command runStartingTrajectory(final AutoTrajectory startingTrajectory) {
        return sequence(
                startingTrajectory.resetOdometry(),
                startingTrajectory.cmd()
        ).withName("RunStartingTrajectory");
    }

    private Supplier<ShotParameters> staticParameters(final Supplier<Pose2d> robotPoseSupplier) {
        return StaticShot.parametersSupplier(
                robotPoseSupplier,
                superstructure::getTurretTranslation,
                swerve::getRobotRelativeSpeeds,
                FieldConstants::getHubPose
        );
    }

    private Supplier<ShotParameters> staticParametersFromPose(final Pose2d pose) {
        return staticParameters(() -> pose);
    }

    private Supplier<ShotParameters> staticParametersFromFinalPose(final AutoTrajectory trajectory) {
        return trajectory.getFinalPose()
                .map(this::staticParametersFromPose)
                .orElse(staticShot);
    }

    private Command intakeFromTrench(
            final Supplier<ShotParameters> fixed,
            final Supplier<ShotParameters> tracking
    ) {
        return parallel(
                intake.intake(),
                sequence(
                        waitUntil(targetIsHub.negate()
                                .and(turretSafe)),
                        superstructure.runParametersHoodStowed(fixed)
                                .until(targetIsHub.and(turretSafe)),
                        superstructure.runParameters(tracking)
                                .onlyIf(turretSafe)
                )
        ).withName("IntakeFromTrench");
    }

    private Command shootStatic() {
        return deadline(
                repeatingSequence(
                        waitUntil(robotStopped
                                .and(superstructure::atSetpoint)),
                        deadline(
                                indexer.toFeed()
                                        .onlyWhile(robotStopped
                                                .and(superstructure::atSetpoint)),
                                intake.stowFeed()
                        )
                ).onlyWhile(fuelState.hasFuel),
                superstructure.runParameters(staticShot)
                        .onlyIf(turretSafe),
                swerve.runWheelXCommand()
        ).withName("ShootStatic");
    }

    private Command shootMovingTOF() {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        deadline(
                                indexer.toFeed()
                                        .onlyWhile(superstructure::atSetpoint),
                                intake.stowFeed()
                        )
                ).onlyWhile(fuelState.hasFuel),
                superstructure.runParameters(movingTOFShot)
                        .onlyIf(turretSafe)
        ).withName("ShootMovingTOF");
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                waitUntil(RobotModeTriggers.autonomous().negate())
        );

        return routine;
    }

    public AutoRoutine upAndAtEm() {
        final AutoRoutine routine = autoFactory.newRoutine("UpAndAtEm");
        final AutoTrajectory upAndAtEm_0 = routine.trajectory("UpAndAtEm_0");
        final AutoTrajectory upAndAtEm_1 = routine.trajectory("UpAndAtEm_1");

        routine.active().onTrue(runStartingTrajectory(upAndAtEm_0));

        upAndAtEm_0.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(upAndAtEm_0),
                        staticShot
                )
        );

        upAndAtEm_0.done().onTrue(sequence(
                shootStatic(),
                deadline(
                        waitUntil(superstructure.safeForTrench)
                                .andThen(upAndAtEm_1.cmd())
                                .asProxy(),
                        superstructure.runParametersHoodStowed(staticShot).asProxy()
                )
        ));

        upAndAtEm_1.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(upAndAtEm_1),
                        staticShot
                )
        );

        upAndAtEm_1.done().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine downAndAtEm() {
        final AutoRoutine routine = autoFactory.newRoutine("DownAndAtEm");
        final AutoTrajectory downAndAtEm_0 = routine.trajectory("DownAndAtEm_0");
        final AutoTrajectory downAndAtEm_1 = routine.trajectory("DownAndAtEm_1");

        routine.active().onTrue(runStartingTrajectory(downAndAtEm_0));

        downAndAtEm_0.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(downAndAtEm_0),
                        staticShot
                )
        );

        downAndAtEm_0.done().onTrue(sequence(
                shootStatic(),
                deadline(
                        waitUntil(superstructure.safeForTrench)
                                .andThen(downAndAtEm_1.cmd())
                                .asProxy(),
                        superstructure.runParametersHoodStowed(staticShot).asProxy()
                )
        ));

        downAndAtEm_1.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(downAndAtEm_1),
                        staticShot
                )
        );

        downAndAtEm_1.done().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine catchMeIfYouCan() {
        final AutoRoutine routine = autoFactory.newRoutine("CatchMeIfYouCan");
        final AutoTrajectory catchMeIfYouCan = routine.trajectory("CatchMeIfYouCan");

        final Supplier<ShotParameters> catchParameters = () -> new ShotParameters(
                new ShotParameters.Shooter(22.5, 0.02),
                Rotation2d.kZero,
                0
        );

        routine.active().onTrue(parallel(
                runStartingTrajectory(catchMeIfYouCan),
                runOnce(() -> fuelState.setSimFuelCount(1)))
        );

        catchMeIfYouCan.active().whileTrue(
                deadline(
                        sequence(
                                waitUntil(turretSafe
                                        .and(superstructure::atSetpoint)),
                                indexer.toFeed()
                                        .onlyWhile(fuelState.hasFuel)
                        ),
                        sequence(
                                superstructure.runParametersHoodStowed(catchParameters)
                                        .until(targetIsHub.negate()
                                                .and(turretSafe)),
                                superstructure.runParameters(catchParameters)
                                        .onlyIf(turretSafe)
                        )
                )
        );

        catchMeIfYouCan.done().onTrue(swerve.runWheelXCommand());

        return routine;
    }
}
