package frc.robot.subsystems.indexer.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;

public class Feeder extends SubsystemBase {
    protected static final String LogKey = "Feeder";
    private static final double VelocityToleranceRotsPerSec = 0.1;

    public enum Goal {
        IDLE(1.2),
        FEED(5);

        public final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private enum InternalGoal {
        NONE,
        IDLE(Goal.IDLE),
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

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged inputs;

    private InternalGoal desiredGoal = InternalGoal.IDLE;
    private InternalGoal currentGoal = InternalGoal.NONE;

    private double velocitySetpointRotsPerSec;

    public Feeder(final Constants.RobotMode mode, final HardwareConstants.FeederConstants constants) {
        this.feederIO = switch (mode) {
            case REAL -> new FeederIOReal(constants);
            case SIM -> new FeederIOSim(constants);
            case REPLAY, DISABLED -> new FeederIO() {};
        };

        this.inputs = new FeederIOInputsAutoLogged();

        this.feederIO.config();
    }

    @Override
    public void periodic() {
        final double shooterPeriodicUpdateStart = Timer.getFPGATimestamp();

        feederIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (MathUtil.isNear(
                velocitySetpointRotsPerSec,
                inputs.rollerVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        )) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/VelocitySetpointRotsPerSec", velocitySetpointRotsPerSec);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - shooterPeriodicUpdateStart)
        );
    }

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> {
                    desiredGoal = InternalGoal.fromGoal(goal);
                    velocitySetpointRotsPerSec = goal.velocityRotsPerSec;
                    feederIO.toFeederVelocity(velocitySetpointRotsPerSec);
                },
                () -> desiredGoal = InternalGoal.IDLE
        );
    }
}
