package frc.robot.subsystems.indexer.feeder;

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

public class Feeder extends SubsystemExt {
    protected static final String LogKey = "Feeder";
    private static final double VelocityToleranceRotsPerSec = 0.1;

    public enum Goal {
        OFF(0),
        FEED(5);

        public final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private enum InternalGoal {
        NONE,
        OFF(Goal.OFF),
        FEED(Goal.FEED);

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

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    private InternalGoal desiredGoal = InternalGoal.OFF;
    private double velocitySetpointRotsPerSec;

    public final LoggedTrigger atSetpoint = group.t(
            "AtSetpoint",
            () -> MathUtil.isNear(
                    velocitySetpointRotsPerSec,
                    inputs.rollerVelocityRotsPerSec,
                    VelocityToleranceRotsPerSec
            )
    );

    public Feeder(final Constants.RobotMode mode, final HardwareConstants.FeederConstants constants) {
        this.feederIO = switch (mode) {
            case REAL -> new FeederIOReal(constants);
            case SIM -> new FeederIOSim(constants);
            case REPLAY, DISABLED -> new FeederIO() {};
        };

        this.feederIO.config();
    }

    @Override
    public void periodic() {
        final double feederPeriodicUpdateStart = Timer.getFPGATimestamp();

        feederIO.updateInputs(inputs);
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
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - feederPeriodicUpdateStart)
        );
    }

    public boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    public boolean isTOFDetected() {
        return inputs.tofDetected;
    }

    public void setTOFDetected(final boolean isDetected) {
        feederIO.setTOFDetected(isDetected);
    }

    private void setVelocityImpl(final double velocityRotsPerSec) {
        velocitySetpointRotsPerSec = velocityRotsPerSec;
        feederIO.toFeederVelocity(velocityRotsPerSec);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setVelocityImpl(goal.velocityRotsPerSec);
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.OFF)
        );
    }
}
