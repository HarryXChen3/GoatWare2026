package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";

    public enum Goal {
        IDLE(Hood.Goal.STOW, Shooter.Goal.IDLE, Turret.Goal.IDLE),
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
        IDLE(Goal.IDLE),
        CLIMB(Goal.CLIMB),
        TRACK_HUB,
        TRACK_HANGAR;

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

    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;

    private InternalGoal desiredGoal = InternalGoal.IDLE;
    private InternalGoal currentGoal = InternalGoal.NONE;

    public Superstructure(final Hood hood, final Shooter shooter, final Turret turret) {
        this.hood = hood;
        this.shooter = shooter;
        this.turret = turret;
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

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    public Command toGoal(final Goal goal) {
        return Commands.parallel(
                Commands.runOnce(() -> desiredGoal = InternalGoal.fromGoal(goal)),
                hood.toGoal(goal.hoodGoal),
                shooter.toGoal(goal.shooterGoal),
                turret.toGoal(goal.turretGoal)
        );
    }

    public Command trackHub() {
        return Commands.parallel(
                hood.runPosition(() -> 0.05),
                turret.runPosition(() -> 1),
                shooter.runVelocity(() -> 20)
        );
    }
}
