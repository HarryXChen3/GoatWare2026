package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Turret extends SubsystemExt {
    protected static final String LogKey = "Turret";
    private static final double PositionToleranceRots = 0.025;
    private static final double VelocityToleranceRotsPerSec = 0.25;

    public enum Goal {
        STOW(0),
        CLIMB(0.5);

        public final double positionRots;

        Goal(final double positionRots) {
            this.positionRots = positionRots;
        }
    }

    private enum InternalGoal {
        NONE,
        STOW(Goal.STOW),
        CLIMB(Goal.CLIMB),
        TRACKING;

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

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);
    private final HardwareConstants.TurretConstants constants;

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private InternalGoal desiredGoal = InternalGoal.STOW;

    private double positionSetpointRots;
    private double velocitySetpointRotsPerSec;

    public final LoggedTrigger atSetpoint = group.t(
            "AtSetpoint",
            () -> MathUtil.isNear(
                    positionSetpointRots,
                    inputs.motorPositionRots,
                    PositionToleranceRots
            ) && MathUtil.isNear(
                    velocitySetpointRotsPerSec,
                    inputs.motorVelocityRotsPerSec,
                    VelocityToleranceRotsPerSec
            )
    );

    public Turret(final Constants.RobotMode mode, final HardwareConstants.TurretConstants constants) {
        this.constants = constants;
        this.turretIO = switch (mode) {
            case REAL -> new TurretIOReal(constants);
            case SIM -> new TurretIOSim(constants);
            case REPLAY, DISABLED -> new TurretIO() {};
        };

        this.turretIO.config();
        this.turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        final Rotation2d absolutePosition = CRT.findAbsolutePosition(
                constants.drivenTurretGearTeeth(),
                inputs.primaryCANcoderPositionRots,
                constants.primaryCANcoderGearTeeth(),
                inputs.secondaryCANcoderPositionRots,
                constants.secondaryCANcoderGearTeeth()
        );
        this.turretIO.seedTurretPosition(absolutePosition);
    }

    @Override
    public void periodic() {
        final double turretPeriodicUpdateStart = Timer.getFPGATimestamp();

        turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        final InternalGoal currentGoal;
        if (atSetpoint()) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);
        Logger.recordOutput(LogKey + "/VelocitySetpointRotsPerSec", velocitySetpointRotsPerSec);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - turretPeriodicUpdateStart)
        );
    }

    public Transform2d getOffsetFromCenter() {
        return constants.offsetFromCenter();
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(inputs.motorPositionRots);
    }

    public boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    private double optimizeWrap(final double targetPositionRots) {
        final double forwardLimitRots = constants.forwardLimitRots();
        final double reverseLimitRots = constants.reverseLimitRots();
        final double currentPositionRots = inputs.motorPositionRots;

        final double wrappedPositionRots = currentPositionRots + MathUtil.inputModulus(
                targetPositionRots - currentPositionRots,
                -0.5,
                0.5
        );

        if (wrappedPositionRots >= reverseLimitRots && wrappedPositionRots <= forwardLimitRots) {
            return wrappedPositionRots;
        }

        final double fullRotationForward = wrappedPositionRots + 1.0;
        final double fullRotationBackward = wrappedPositionRots - 1.0;

        final boolean isForwardWithinBound = fullRotationForward >= reverseLimitRots
                && fullRotationForward <= forwardLimitRots;
        final boolean isBackwardWithinBound = fullRotationBackward >= reverseLimitRots
                && fullRotationBackward <= forwardLimitRots;

        if (isForwardWithinBound && isBackwardWithinBound) {
            final double forwardTravelRots = Math.abs(fullRotationForward - currentPositionRots);
            final double backwardTravelRots = Math.abs(fullRotationBackward - currentPositionRots);

            return forwardTravelRots <= backwardTravelRots ? fullRotationForward : fullRotationBackward;
        } else if (isForwardWithinBound) {
            return fullRotationForward;
        } else if (isBackwardWithinBound) {
            return fullRotationBackward;
        }

        return MathUtil.clamp(wrappedPositionRots, reverseLimitRots, forwardLimitRots);
    }

    private void setPositionImpl(final double positionRots, final double velocityRotsPerSec) {
        positionSetpointRots = optimizeWrap(positionRots);
        velocitySetpointRotsPerSec = velocityRotsPerSec;
        turretIO.trackTurretPosition(positionSetpointRots, velocityRotsPerSec);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        positionSetpointRots = goal.positionRots;
        velocitySetpointRotsPerSec = 0;
        turretIO.toTurretPosition(positionSetpointRots);
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.STOW)
        );
    }

    public Command runGoal(final Goal goal) {
        return startIdle(() -> setGoalImpl(goal));
    }

    public Command toPosition(
            final Supplier<Rotation2d> positionSupplier,
            final DoubleSupplier velocitySupplier
    ) {
        return instantRunEnd(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setPositionImpl(positionSupplier.get().getRotations(), velocitySupplier.getAsDouble()),
                () -> setGoalImpl(Goal.STOW)
        );
    }

    public Command runPosition(
            final Supplier<Rotation2d> positionSupplier,
            final DoubleSupplier velocitySupplier
    ) {
        return instantRun(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setPositionImpl(positionSupplier.get().getRotations(), velocitySupplier.getAsDouble())
        );
    }
}
