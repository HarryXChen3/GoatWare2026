package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.ShootCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.FuelState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.params.MovingTOFShot;
import frc.robot.subsystems.superstructure.params.ShotParameters;
import frc.robot.subsystems.superstructure.params.ShotProvider;
import frc.robot.subsystems.superstructure.params.StaticShot;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import frc.robot.utils.commands.trigger.RobotModeLoggedTriggers;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Autos {
    public static final String LogKey = "Auto";

    @SuppressWarnings("FieldCanBeLocal")
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final FuelState fuelState;
    private final Superstructure superstructure;

    private final AutoFactory autoFactory;

    private final ShotProvider<ShotProvider.Kind.Static> staticShotProvider;
    private final Supplier<ShotParameters> staticShot;

    private final ShotProvider<ShotProvider.Kind.Moving> movingShotProvider;
    private final Supplier<ShotParameters> movingShot;

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

        this.staticShotProvider = new StaticShot();
        this.staticShot = staticParameters(swerve::getPose);

        this.movingShotProvider = new MovingTOFShot();
        this.movingShot = movingParameters(FieldConstants::getHubPose);

        this.robotStopped = group.t(
                "RobotStopped",
                () -> ShootCommands.linearSpeed(swerve.getFieldRelativeSpeeds()) <= 0.01
        );
        this.targetIsHub = group.t(
                "TargetIsHub",
                () -> ShootCommands.getTarget(superstructure.getTurretTranslation(swerve.getPose()))
                        == ShootCommands.Target.HUB
        );
        this.turretSafe = group.t(
                "TurretSafe",
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
        return staticShotProvider.parametersSupplier(
                robotPoseSupplier,
                superstructure::getRobotToTurret,
                swerve::getRobotRelativeSpeeds,
                FieldConstants::getHubPose
        );
    }

    private Supplier<ShotParameters> movingParameters(final Supplier<Pose2d> targetPoseSupplier) {
        return movingShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToTurret,
                swerve::getRobotRelativeSpeeds,
                targetPoseSupplier
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
                                intake.stowFeed().asProxy()
                                        .unless(intake.isIntaking)
                        )
                ).onlyWhile(fuelState.hasFuel
                        .or(intake.isIntaking)),
                superstructure.runParameters(movingShot)
                        .onlyIf(turretSafe)
        ).withName("ShootMovingTOF");
    }

    private Command ferryToPoseMovingTOF(final Pose2d ferryTo) {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        deadline(
                                indexer.toFeed()
                                        .onlyWhile(superstructure::atSetpoint),
                                intake.stowFeed().asProxy()
                                        .unless(intake.isIntaking)
                        )
                ).onlyWhile(fuelState.hasFuel
                        .or(intake.isIntaking)),
                superstructure.runParameters(movingParameters(() -> ferryTo))
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

        routine.active().onTrue(parallel(
                runStartingTrajectory(upAndAtEm_0),
                runOnce(fuelState::setSimFuelPreloaded)
        ));
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

        routine.active().onTrue(parallel(
                runStartingTrajectory(downAndAtEm_0),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

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

    public AutoRoutine upFerryAndScoot() {
        final AutoRoutine routine = autoFactory.newRoutine("UpFerryAndScoot");
        final AutoTrajectory upFerryAndScoot_0 = routine.trajectory("UpFerryAndScoot_0");

        routine.active().onTrue(parallel(
                runStartingTrajectory(upFerryAndScoot_0),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

        upFerryAndScoot_0.active().whileTrue(
                parallel(
                        intake.intake(),
                        sequence(
                                waitUntil(targetIsHub.negate()
                                        .and(turretSafe)),
                                ferryToPoseMovingTOF(FieldConstants.getFerryLeft())
                                        .until(targetIsHub.or(turretSafe.negate())),
                                superstructure.runParametersHoodStowed(movingShot)
                                        .until(targetIsHub.and(turretSafe)),
                                shootMovingTOF()
                        )
                )
        );

        upFerryAndScoot_0.done().onTrue(sequence(
                shootStatic()
        ));

        return routine;
    }

    public AutoRoutine catchMeIfYouCan() {
        final AutoRoutine routine = autoFactory.newRoutine("CatchMeIfYouCan");
        final AutoTrajectory catchMeIfYouCan = routine.trajectory("CatchMeIfYouCan");

        final Supplier<ShotParameters> catchParameters = () -> new ShotParameters(
                new ShotParameters.Shooter(22.5, 0.014),
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

    public Command warmup() {
        final AutoRoutine routine = autoFactory.newRoutine("Warmup");
        final AutoTrajectory warmup = routine.trajectory("Warmup");

//        warmup.active().whileTrue(
//                intakeFromTrench(
//                        staticParametersFromFinalPose(doohickey),
//                        staticShot
//                )
//        );
//
//        warmup.done().onTrue(
//                sequence(
//                        waitUntil(targetIsHub),
//                        shootStatic()
//                )
//        );

        final Field pollCountField;
        final Field cycleTimestampField;
        final Field isActiveField;
        try {
            pollCountField = AutoRoutine.class.getDeclaredField("pollCount");
            pollCountField.setAccessible(true);

            cycleTimestampField = AutoRoutine.class.getDeclaredField("cycleTimestamp");
            cycleTimestampField.setAccessible(true);

            isActiveField = AutoRoutine.class.getDeclaredField("isActive");
            isActiveField.setAccessible(true);
        } catch (final Exception e) {
            DriverStation.reportError(
                    String.format("Could not access AutoRoutine fields\nReason: %s", e),
                    true
            );

            return none();
        }

        final EventLoop loop = routine.loop();
        final LoggedTrigger.Group routineGroup = LoggedTrigger.Group.from("Doohickey", loop);
        final LoggedTrigger routineActive = routineGroup.t("RoutineActive", () -> {
            try {
                return (boolean) isActiveField.get(routine);
            } catch (final IllegalAccessException e) {
                // drop
                return false;
            }
        });

        final Command warmupCommand;
        try {
            final Method cmdInitialize = AutoTrajectory.class.getDeclaredMethod("cmdInitialize");
            cmdInitialize.setAccessible(true);

            final Method cmdExecute = AutoTrajectory.class.getDeclaredMethod("cmdExecute");
            cmdExecute.setAccessible(true);

            final Method cmdEnd = AutoTrajectory.class.getDeclaredMethod("cmdEnd", boolean.class);
            cmdEnd.setAccessible(true);

            final Field activeTimer = AutoTrajectory.class.getDeclaredField("activeTimer");
            activeTimer.setAccessible(true);

            warmupCommand = new FunctionalCommand(
                    () -> {
                        try {
                            cmdInitialize.invoke(warmup);
                        } catch (final Exception e) {
                            // drop
                        }
                    },
                    () -> {
                        try {
                            cmdExecute.invoke(warmup);
                        } catch (final Exception e) {
                            // drop
                        }
                    },
                    interrupted -> {
                        try {
                            cmdEnd.invoke(warmup, interrupted);
                        } catch (final Exception e) {
                            // drop
                        }
                    },
                    () -> {
                        try {
                            return ((Timer) activeTimer.get(warmup)).get()
                                    > warmup.getRawTrajectory().getTotalTime();
                        } catch (final Exception e) {
                            // drop
                            return false;
                        }
                    }
            )
                    .finallyDo(swerve::stoppedZero)
                    .ignoringDisable(true);
        } catch (final Exception e) {
            DriverStation.reportError(
                    String.format("Could not access AutoTrajectory fields\nReason: %s", e),
                    true
            );

            return none();
        }

        routineActive
                .whileTrue(warmupCommand);

        final LoggedTrigger disabled = RobotModeLoggedTriggers.disabled(group);
        return run(() -> {
            try {
                pollCountField.set(routine, ((int) pollCountField.get(routine)) + 1);
                cycleTimestampField.set(routine, Timer.getTimestamp());
                loop.poll();
                isActiveField.set(routine, true);
            } catch (final IllegalAccessException e) {
                // drop
            }
        })
                .finallyDo(() -> {
                    warmupCommand.cancel();
                    routine.reset();
                })
                .onlyIf(disabled)
                .onlyWhile(disabled)
                .withTimeout(20)
                .ignoringDisable(true);
    }
}
