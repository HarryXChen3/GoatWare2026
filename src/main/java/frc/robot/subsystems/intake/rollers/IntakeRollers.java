package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.SubsystemExt;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;

public class IntakeRollers extends SubsystemExt {
    protected static final String LogKey = "IntakeRollers";
    private static final double VelocityToleranceRotsPerSec = 0.5;

    public enum Goal {
        OFF(0),
        INTAKE(10);

        public final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private enum InternalGoal {
        NONE,
        OFF(Goal.OFF),
        INTAKE(Goal.INTAKE);

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

    private final IntakeRollersIO intakeRollersIO;
    private final IntakeRollersIOInputsAutoLogged inputs;

    private InternalGoal desiredGoal = InternalGoal.OFF;
    private InternalGoal currentGoal = InternalGoal.NONE;
    private double velocitySetpointRotsPerSec;

    public IntakeRollers(final Constants.RobotMode mode, final HardwareConstants.IntakeRollersConstants constants) {
        this.intakeRollersIO = switch (mode) {
            case REAL -> new IntakeRollersIOReal(constants);
            case SIM -> new IntakeRollersIOSim(constants);
            case REPLAY, DISABLED -> new IntakeRollersIO() {};
        };

        this.inputs = new IntakeRollersIOInputsAutoLogged();

        this.intakeRollersIO.config();
    }

    @Override
    public void periodic() {
        final double intakeRollersPeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeRollersIO.updateInputs(inputs);
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
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakeRollersPeriodicUpdateStart)
        );
    }

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    private void setVelocityImpl(final double velocityRotsPerSec) {
        velocitySetpointRotsPerSec = velocityRotsPerSec;
        intakeRollersIO.toIntakeVelocity(velocityRotsPerSec);
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
                () -> setGoalImpl(Goal.OFF)
        );
    }
}
