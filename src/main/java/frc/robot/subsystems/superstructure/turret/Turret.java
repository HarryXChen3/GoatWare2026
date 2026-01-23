package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {
    protected static final String LogKey = "Turret";
    private static final double PositionToleranceRots = 0.1;
    private static final double VelocityToleranceRotsPerSec = 0.05;

    public enum Goal {
        IDLE(0),
        CLIMB(0.5);

        public final double positionRots;

        Goal(final double positionRots) {
            this.positionRots = positionRots;
        }
    }

    private enum InternalGoal {
        NONE,
        IDLE(Goal.IDLE),
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

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs;

    private InternalGoal desiredGoal = InternalGoal.IDLE;
    private InternalGoal currentGoal = InternalGoal.NONE;

    private double positionSetpointRots;

    public Turret(final Constants.RobotMode mode, final HardwareConstants.TurretConstants constants) {
        this.turretIO = switch (mode) {
            case REAL -> new TurretIOReal(constants);
            case SIM -> new TurretIOSim(constants);
            case REPLAY, DISABLED -> new TurretIO() {};
        };

        this.inputs = new TurretIOInputsAutoLogged();

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

        if (MathUtil.isNear(positionSetpointRots, inputs.motorPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.motorVelocityRotsPerSec, VelocityToleranceRotsPerSec)
        ) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - turretPeriodicUpdateStart)
        );
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(inputs.motorPositionRots);
    }

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    private void setPositionImpl(final double positionRots) {
        positionSetpointRots = positionRots;
        turretIO.toTurretPosition(positionSetpointRots);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setPositionImpl(goal.positionRots);
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.IDLE)
        );
    }

    public Command runGoal(final Goal goal) {
        return run(() -> setGoalImpl(goal));
    }

    public Command toPosition(final DoubleSupplier positionRotsSupplier) {
        return runEnd(
                () -> {
                    desiredGoal = InternalGoal.TRACKING;
                    setPositionImpl(positionRotsSupplier.getAsDouble());
                },
                () -> setGoalImpl(Goal.IDLE)
        );
    }
}
