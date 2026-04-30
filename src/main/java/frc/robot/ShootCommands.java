package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.FuelState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.params.MovingTOFShot;
import frc.robot.subsystems.superstructure.params.ShotParameters;
import frc.robot.subsystems.superstructure.params.ShotProvider;
import frc.robot.subsystems.superstructure.params.StaticShot;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommands extends VirtualSubsystem {
    protected static final String LogKey = "ShootCommands";
    private static final SwerveSpeed.Speeds ShootAndScootSpeeds = SwerveSpeed.Speeds.SHOOT_AND_SCOOT;
    private static final double ShootAndScootTolerance = 0.25;
    public enum Target {
        HUB,
        FERRY,
        NONE_FERRY_BLOCKED
    }

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final FuelState fuelState;
    private final Superstructure superstructure;

    private final Supplier<Target> targetSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;

    private final ShotProvider<ShotProvider.Kind.Static> staticShotProvider;
    private final Supplier<ShotParameters> staticShot;

    private final ShotProvider<ShotProvider.Kind.Moving> movingShotProvider;
    private final Supplier<ShotParameters> movingShot;

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

        this.targetSupplier = () -> getTarget(superstructure.getShooterPose(swerve.getPose()).getTranslation());
        this.targetPoseSupplier = getTargetPoseSupplier();

        this.staticShotProvider = new StaticShot();
        this.staticShot = staticShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToShooter,
                swerve::getRobotRelativeSpeeds,
                targetPoseSupplier
        );

        this.movingShotProvider = new MovingTOFShot();
        this.movingShot = movingShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToShooter,
                swerve::getRobotRelativeSpeeds,
                targetPoseSupplier
        );
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/Target", targetSupplier.get());
    }

    public static double linearSpeed(final ChassisSpeeds speeds) {
        return Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
        );
    }

    public static Target getTarget(final Translation2d shooterTranslation) {
        final double shooterX = shooterTranslation.getX();
        final double shooterY = shooterTranslation.getY();

        final double ferryXBoundary = FieldConstants.getFerryXBoundary();
        final boolean isRed = Robot.IsRedAlliance.getAsBoolean();
        final boolean canFerryX = isRed
                ? shooterX <= ferryXBoundary
                : shooterX >= ferryXBoundary;

        final double ferryLeftBoundary = FieldConstants.getFerryLeftYBoundary();
        final double ferryRightBoundary = FieldConstants.getFerryRightYBoundary();
        final boolean canFerryY = isRed
                ? (shooterY >= ferryLeftBoundary || shooterY <= ferryRightBoundary)
                : (shooterY <= ferryLeftBoundary || shooterY >= ferryRightBoundary);

        return canFerryX
                ? (canFerryY ? Target.FERRY : Target.NONE_FERRY_BLOCKED)
                : Target.HUB;
    }

    public static Supplier<Pose2d> getTargetPoseSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Function<Pose2d, Target> targetFunction
    ) {
        return () -> {
            final Pose2d robotPose = robotPoseSupplier.get();
            final Target target = targetFunction.apply(robotPose);
            return switch (target) {
                case HUB -> FieldConstants.getHubPose();
                case FERRY, NONE_FERRY_BLOCKED -> {
                    final boolean isRed = Robot.IsRedAlliance.getAsBoolean();
                    final Pose2d ferryLeft = FieldConstants.getFerryLeft();
                    final Pose2d ferryRight = FieldConstants.getFerryRight();

                    yield robotPose.getY() <= FieldConstants.getFerryLeftYBoundary()
                            ? (isRed ? ferryRight : ferryLeft)
                            : (isRed ? ferryLeft : ferryRight);
                }
            };
        };
    }

    private Supplier<Pose2d> getTargetPoseSupplier() {
        return getTargetPoseSupplier(
                swerve::getPose,
                robotPose -> ShootCommands.getTarget(superstructure.getShooterPose(robotPose).getTranslation())
        );
    }

    public Command trackTarget() {
        final Supplier<ShotParameters> staticParametersSupplier = staticShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToShooter,
                swerve::getRobotRelativeSpeeds,
                targetPoseSupplier
        );
        final Supplier<ShotParameters> movingTOFParametersSupplier = movingShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToShooter,
                () -> {
                    final ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();
                    final double linearSpeed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
                    return robotSpeeds
                            .div(linearSpeed)
                            .times(Math.min(linearSpeed, ShootAndScootSpeeds.getTranslationSpeed()));
                },
                targetPoseSupplier
        );

        return superstructure.runParametersHoodStowed(
                () -> linearSpeed(swerve.getFieldRelativeSpeeds()) <= 1e-3
                        ? staticParametersSupplier.get()
                        : movingTOFParametersSupplier.get()
        ).withName("TrackTarget");
    }

    public Command stopAndShoot() {
        final LoggedTrigger targetValid = group.t(
                "TargetValid",
                () -> switch (targetSupplier.get()) {
                    case HUB, FERRY -> true;
                    case NONE_FERRY_BLOCKED -> false;
                });

        final ShotParameters.CachedShot cachedShot = ShotParameters.getCached(staticShot);

        return deadline(
                repeatingSequence(
                        waitUntil(targetValid
                                .and(superstructure::atSetpoint)
                                .and(swerve.atHeadingSetpoint)),
                        deadline(
                                indexer.toFeed()
                                        .onlyWhile(targetValid
                                                .and(superstructure::atSetpoint)
                                                .and(swerve.atHeadingSetpoint)),
                                intake.stowFeed()
                        )
                )
                        .onlyIf(fuelState.hasFuel)
                        .onlyWhile(fuelState.hasFuel
                                .or(intake.isIntaking)),
                superstructure.runParameters(cachedShot),
                swerve.faceAngle(() -> cachedShot.get().robotAngle()),
                cachedShot.clear()
        ).withName("StopAndShoot");
    }

    public Command shoot(final DoubleSupplier xSpeedSupplier, final DoubleSupplier ySpeedSupplier) {
        final LoggedTrigger targetValid = group.t(
                "TargetValid",
                () -> switch (targetSupplier.get()) {
                    case HUB, FERRY -> true;
                    case NONE_FERRY_BLOCKED -> false;
                });
        final LoggedTrigger swerveReady = group.t(
                "SwerveReady",
                () -> linearSpeed(swerve.getFieldRelativeSpeeds())
                        <= ShootAndScootSpeeds.getTranslationSpeed() + ShootAndScootTolerance
        );

        final ShotParameters.CachedShot cachedShot = ShotParameters.getCached(movingShot);
        return deadline(
                repeatingSequence(
                        waitUntil(targetValid
                                .and(swerveReady)
                                .and(superstructure::atSetpoint)
                                .and(swerve.atHeadingSetpoint)),
                        Commands.deadline(
                                indexer.toFeed()
                                        .onlyWhile(targetValid
                                                .and(swerveReady)
                                                .and(superstructure::atSetpoint)
                                                .and(swerve.atHeadingSetpoint)),
                                intake.stowFeed().asProxy()
                                        .unless(intake.isIntaking)
                        )
                )
                        .onlyIf(fuelState.hasFuel)
                        .onlyWhile(fuelState.hasFuel
                                .or(intake.isIntaking)),
                SwerveSpeed.toSwerveSpeed(ShootAndScootSpeeds),
                superstructure.runParameters(cachedShot),
                swerve.teleopFacingAngle(
                        xSpeedSupplier,
                        ySpeedSupplier,
                        () -> cachedShot.get().robotAngle(),
                        () -> cachedShot.get().robotOmegaRadsPerSec()
                ),
                cachedShot.clear()
        ).withName("Shoot");
    }
}