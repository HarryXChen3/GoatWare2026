package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.utils.Container;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";
    protected static final LoggedTrigger.Group Group = LoggedTrigger.Group.from(LogKey);

    public enum Goal {
        STOW(Hood.Goal.STOW, Shooter.Goal.IDLE, Turret.Goal.IDLE),
        CLIMB(Hood.Goal.STOW, Shooter.Goal.OFF, Turret.Goal.CLIMB);

        public final Hood.Goal hoodGoal;
        public final Shooter.Goal shooterGoal;
        public final Turret.Goal turretGoal;

        Goal(final Hood.Goal hoodGoal, final Shooter.Goal shooterGoal, final Turret.Goal turretGoal) {
            this.hoodGoal = hoodGoal;
            this.shooterGoal = shooterGoal;
            this.turretGoal = turretGoal;
        }
    }

    private enum InternalGoal {
        NONE,
        STOW(Goal.STOW),
        CLIMB(Goal.CLIMB),
        DYNAMIC;

        public static final HashMap<Goal, InternalGoal> GoalToInternal = new HashMap<>();
        static {
            for (final InternalGoal goal : InternalGoal.values()) {
                if (goal.goal != null) {
                    GoalToInternal.put(goal.goal, goal);
                }
            }
        }

        public static InternalGoal fromGoal(final Goal goal) {
            return Objects.requireNonNull(GoalToInternal.get(goal));
        }

        public final Goal goal;
        InternalGoal(final Goal goal) {
            this.goal = goal;
        }

        InternalGoal() {
            this(null);
        }
    }

    private final Turret turret;
    private final Shooter shooter;
    private final Hood hood;

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private InternalGoal currentGoal = InternalGoal.NONE;

    public Superstructure(final Turret turret, final Shooter shooter, final Hood hood) {
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
    }

    @Override
    public void periodic() {
        final double superstructurePeriodicUpdateStart = Timer.getFPGATimestamp();

        if (hood.atSetpoint() && shooter.atSetpoint() && turret.atSetpoint()) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - superstructurePeriodicUpdateStart)
        );
    }

    public Transform2d getOffsetFromCenter() {
        return turret.getOffsetFromCenter();
    }

    public Pose3d[] getComponentPoses() {
        final Pose3d turretPose = new Pose3d(
                SimConstants.Turret.ORIGIN_OFFSET,
                new Rotation3d(turret.getPosition())
        );

        final Pose3d hoodPose = turretPose
                .plus(new Transform3d(
                        SimConstants.Hood.TURRET_OFFSET,
                        new Rotation3d(0, hood.getPosition().getRadians(), 0)
                ));

        return new Pose3d[] {
                turretPose,
                hoodPose
        };
    }

    private Command updateDesiredGoal(final Supplier<InternalGoal> goalSupplier) {
        return Commands.runOnce(() -> desiredGoal = goalSupplier.get());
    }

    private Command updateDesiredGoal(final InternalGoal goal) {
        return Commands.runOnce(() -> desiredGoal = goal);
    }

    private Command toGoalLike(final InternalGoal goal, final Command... commands) {
        return updateDesiredGoal(goal)
                .alongWith(commands)
                .finallyDo(() -> desiredGoal = InternalGoal.STOW);
    }

    private boolean atGoal(final InternalGoal goal) {
        return currentGoal == goal;
    }

    public LoggedTrigger atGoal(final Goal goal) {
        return Group.t("AtGoal", () -> atGoal(InternalGoal.fromGoal(goal)));
    }

    public boolean atSetpoint() {
        return currentGoal == desiredGoal;
    }

    public Command toGoal(final Goal goal) {
        return toGoalLike(
                InternalGoal.fromGoal(goal),
                hood.toGoal(goal.hoodGoal),
                shooter.toGoal(goal.shooterGoal),
                turret.toGoal(goal.turretGoal)
        );
    }

    public Command runGoal(final Goal goal) {
        return Commands.parallel(
                updateDesiredGoal(InternalGoal.fromGoal(goal)),
                hood.runGoal(goal.hoodGoal),
                shooter.runGoal(goal.shooterGoal),
                turret.runGoal(goal.turretGoal)
        );
    }

    public Command waitForGoal(final Goal goal) {
        return runGoal(goal).until(atGoal(goal));
    }

    public Command runParameters(
            final Supplier<ShotParameters> shotParametersSupplier,
            final Supplier<Rotation2d> turretPositionSupplier,
            final DoubleSupplier turretVelocitySupplier
    ) {
        final Container<ShotParameters> parameters = Container.empty();
        final Supplier<ShotParameters> cached = () -> {
            if (parameters.hasValue()) {
                return parameters.get();
            }

            final ShotParameters params = shotParametersSupplier.get();
            parameters.set(params);
            return params;
        };

        return Commands.parallel(
                updateDesiredGoal(InternalGoal.DYNAMIC),
                turret.runPosition(turretPositionSupplier, turretVelocitySupplier),
                hood.runPosition(() -> cached.get().hoodPositionRots()),
                shooter.runVelocity(() -> cached.get().shooterVelocityRotsPerSec()),
                Commands.run(parameters::clear)
        );
    }
}
