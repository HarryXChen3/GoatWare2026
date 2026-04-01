package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.MathUtil;
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

public class Shooter extends SubsystemExt {
    protected static final String LogKey = "Shooter";
    private static final double VelocityToleranceRotsPerSec = 0.5;

    public enum Goal {
        OFF(0),
        IDLE(20);

        public final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private enum InternalGoal {
        NONE,
        OFF(Goal.OFF),
        IDLE(Goal.IDLE),
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

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private InternalGoal desiredGoal = InternalGoal.OFF;
    private double velocitySetpointRotsPerSec;

    public final LoggedTrigger atSetpoint = group.t(
            "AtSetpoint",
            () -> MathUtil.isNear(
                    velocitySetpointRotsPerSec,
                    inputs.masterVelocityRotsPerSec,
                    VelocityToleranceRotsPerSec
            )
    );

    public Shooter(final Constants.RobotMode mode, final HardwareConstants.ShooterConstants constants) {
        this.shooterIO = switch (mode) {
            case REAL -> new ShooterIOReal(constants);
            case SIM -> new ShooterIOSim(constants);
            case REPLAY, DISABLED -> new ShooterIO() {};
        };
    }

    @Override
    public void periodic() {
        final double shooterPeriodicUpdateStart = Timer.getFPGATimestamp();

        shooterIO.updateInputs(inputs);
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
        Logger.recordOutput(LogKey + "/VelocitySetpointRotsPerSec", velocitySetpointRotsPerSec);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - shooterPeriodicUpdateStart)
        );
    }

    public boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    public double getVelocityRotsPerSec() {
        return inputs.masterVelocityRotsPerSec;
    }

    private void setVelocityImpl(final double velocityRotsPerSec) {
        velocitySetpointRotsPerSec = velocityRotsPerSec;
        shooterIO.toShooterVelocity(velocitySetpointRotsPerSec);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setVelocityImpl(goal.velocityRotsPerSec);
    }

    public Command toInstantGoal(final Goal goal) {
        return runOnce(() -> setGoalImpl(goal));
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.IDLE)
        );
    }

    public Command runGoal(final Goal goal) {
        return startIdle(() -> setGoalImpl(goal));
    }

    public Command toVelocity(final DoubleSupplier velocityRotsPerSecSupplier) {
        return instantRunEnd(
                () -> desiredGoal = InternalGoal.DYNAMIC,
                () -> setVelocityImpl(velocityRotsPerSecSupplier.getAsDouble()),
                () -> setGoalImpl(Goal.IDLE)
        );
    }

    public Command runVelocity(final DoubleSupplier velocityRotsPerSecSupplier) {
        return instantRun(
                () -> desiredGoal = InternalGoal.DYNAMIC,
                () -> setVelocityImpl(velocityRotsPerSecSupplier.getAsDouble())
        );
    }
}
