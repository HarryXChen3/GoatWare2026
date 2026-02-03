package frc.robot.subsystems.intake.slide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.commands.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;

public class IntakeSlide extends SubsystemBase {
    protected static final String LogKey = "IntakeSlide";
    protected static final double PositionToleranceRots = 0.05;
    private static final double VelocityToleranceRotsPerSec = 0.1;

    public enum Goal {
        STOW(0),
        INTAKE(3.89);

        public final double positionRots;

        Goal(final double positionRots) {
            this.positionRots = positionRots;
        }
    }

    private enum InternalGoal {
        NONE,
        STOW(Goal.STOW),
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

    public enum HoldMode {
        STIFF,
        SQUISHY
    }

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final IntakeSlideIO intakeSlideIO;
    private final IntakeSlideIOInputsAutoLogged inputs;

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private InternalGoal currentGoal = InternalGoal.NONE;
    private double positionSetpointRots;

    private HoldMode holdMode = HoldMode.STIFF;
    private final LoggedTrigger squishyModeTrigger = group.t("SquishyMode", this::atSetpoint).debounce(0.1);

    public IntakeSlide(final Constants.RobotMode mode, final HardwareConstants.IntakeSlideConstants constants) {
        this.intakeSlideIO = switch (mode) {
            case REAL -> new IntakeSlideIOReal(constants);
            case SIM -> new IntakeSlideIOSim(constants);
            case REPLAY, DISABLED -> new IntakeSlideIO() {};
        };

        this.inputs = new IntakeSlideIOInputsAutoLogged();
        this.intakeSlideIO.config();

        squishyModeTrigger.onTrue(Commands.runOnce(() -> {
            holdMode = HoldMode.SQUISHY;
            setPositionImpl(positionSetpointRots);
        }));
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeSlideIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (MathUtil.isNear(positionSetpointRots, inputs.slidePositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.slideVelocityRotsPerSec, VelocityToleranceRotsPerSec)
        ) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);
        Logger.recordOutput(LogKey + "/HoldMode", holdMode);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakePeriodicUpdateStart)
        );
    }

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    public Pose3d[] getComponentPoses() {
        final Pose3d slideExtended = SimConstants.IntakeSlide.ExtendedPose;
        final Pose3d slideRetracted = SimConstants.IntakeSlide.RetractedPose;

        final Pose3d hopperExtended = SimConstants.HopperExtension.ExtendedPose;
        final Pose3d hopperRetracted = SimConstants.HopperExtension.RetractedPose;

        final double extensionMeters = inputs.slidePositionRots
                * SimConstants.IntakeSlide.SlideRotationsToLinearDistanceMetersRatio;
        final double totalExtensionDistance = slideExtended.getTranslation()
                .getDistance(slideRetracted.getTranslation());
        final double extensionRatio = extensionMeters / totalExtensionDistance;

        return new Pose3d[] {
                slideRetracted.interpolate(slideExtended, extensionRatio),
                hopperRetracted.interpolate(hopperExtended, extensionRatio)
        };
    }

    private void setPositionImpl(final double positionRots) {
        positionSetpointRots = positionRots;
        switch (holdMode) {
            case STIFF -> intakeSlideIO.toSlidePosition(positionRots);
            case SQUISHY -> intakeSlideIO.holdSlidePosition(positionRots);
        }
    }

    private void setGoalImpl(final Goal goal) {
        if (desiredGoal.goal != goal || !atSetpoint()) {
            holdMode = HoldMode.STIFF;
        }

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
