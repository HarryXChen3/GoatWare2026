package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;

public class Climb extends SubsystemBase {
    protected static final String LogKey = "Climb";
    private static final double PositionToleranceRots = 0.05;
    private static final double VelocityToleranceRotsPerSec = 0.1;

    public enum Goal {
        STOW(0, ClimbDirection.DEPLOY),
        READY_CLIMB(5, ClimbDirection.DEPLOY),
        CLIMB(0, ClimbDirection.CLIMB);

        public final double positionRots;
        public final ClimbDirection climbDirection;

        Goal(final double positionRots, final ClimbDirection climbDirection) {
            this.positionRots = positionRots;
            this.climbDirection = climbDirection;
        }
    }

    private enum InternalGoal {
        NONE,
        STOW(Goal.STOW),
        READY_CLIMB(Goal.READY_CLIMB),
        CLIMB(Goal.CLIMB);

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

    public enum ClimbDirection {
        DEPLOY,
        CLIMB
    }

    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs;

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private InternalGoal currentGoal = InternalGoal.NONE;
    private double positionSetpointRots;

    private ClimbDirection climbDirection;

    public Climb(final Constants.RobotMode mode, final HardwareConstants.ClimbConstants constants) {
        this.climbIO = switch (mode) {
            case REAL -> new ClimbIOReal(constants);
            case SIM -> new ClimbIOSim(constants);
            case REPLAY, DISABLED -> new ClimbIO() {};
        };

        this.inputs = new ClimbIOInputsAutoLogged();
        this.climbIO.config();
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = Timer.getFPGATimestamp();

        climbIO.updateInputs(inputs);
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
        Logger.recordOutput(LogKey + "/ClimbDirection", climbDirection);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakePeriodicUpdateStart)
        );
    }

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    public Pose3d[] getComponentPoses() {
        final Pose3d climbExtended = SimConstants.Climb.ExtendedPose;
        final Pose3d climbRetracted = SimConstants.Climb.RetractedPose;

        final double extensionMeters = inputs.motorPositionRots * SimConstants.Climb.PulleyCircumferenceMeters;
        final double totalExtensionDistance = climbExtended.getTranslation()
                .getDistance(climbRetracted.getTranslation());

        final double stage0MaxExtension = SimConstants.Climb.Stage0MaxExtension;
        final double stage0ExtensionRatio = Math.min(extensionMeters / stage0MaxExtension, 1);
        final double stage1ExtensionRatio = Math.max(extensionMeters - stage0MaxExtension, 0)
                / totalExtensionDistance;

        final double stage0MaxExtensionRatio = MathUtil.clamp(
                stage0MaxExtension / totalExtensionDistance,
                0, 1
        );
        final Pose3d stage0MaxRetractionPose =
                climbExtended.interpolate(climbRetracted, 1 - stage0MaxExtensionRatio);

        return new Pose3d[] {
                stage0MaxRetractionPose.interpolate(climbExtended, stage0ExtensionRatio),
                climbRetracted.interpolate(
                        climbExtended,
                        (stage0ExtensionRatio * stage0MaxExtensionRatio) + stage1ExtensionRatio
                )
        };
    }

    private void setPositionImpl(final double positionRots) {
        positionSetpointRots = positionRots;
        switch (climbDirection) {
            case DEPLOY -> climbIO.deployToPosition(positionRots);
            case CLIMB -> climbIO.climbToPosition(positionRots);
        }
    }

    private void setGoalImpl(final Goal goal) {
        climbDirection = goal.climbDirection;
        desiredGoal = InternalGoal.fromGoal(goal);
        setPositionImpl(goal.positionRots);
    }

    public Command toInstantGoal(final Goal goal) {
        return runOnce(() -> setGoalImpl(goal));
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.STOW)
        );
    }
}
