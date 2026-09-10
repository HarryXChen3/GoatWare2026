package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemExt {
    protected static final String LogKey = "IntakePivot";

    private static final double ZeroPositionRots = -0.314453;
    private static final double ZeroingVolts = -1.5;
    private static final double ZeroingCurrentAmps = 80;
    private static final double PositionToleranceRots = 0.01;
    private static final double VelocityToleranceRotsPerSec = 0.1;

    private interface State {
        String name();
    }

    public enum Goal implements State {
        STOW(-0.3, GoalBehavior.none()),
        INTAKE(0.038818, GoalBehavior.none()),
        TRENCH_RETRACT(-0.12, GoalBehavior.none()),
        GYRATE_MAX(-0.12, GoalBehavior.trapezoidal(2.4, 4.6)),
        GYRATE_MIN(-0.054443, GoalBehavior.trapezoidal(2.4, 4.6));

        public final double positionRots;
        public final GoalBehavior behavior;

        Goal(final double positionRots, final GoalBehavior behavior) {
            this.positionRots = positionRots;
            this.behavior = behavior;
        }
    }

    private enum Dynamic implements State {
        NONE,
        HOLD_POSITION,
        ZEROING
    }

    public enum ProfileType {
        NONE,
        TRAPEZOIDAL
    }

    public static class GoalBehavior {
        public final ProfileType type;
        public final double velocity;
        public final double acceleration;

        private GoalBehavior(final ProfileType type, final double velocity, final double acceleration) {
            this.type = type;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        public static GoalBehavior none() {
            return new GoalBehavior(ProfileType.NONE, 0, 0);
        }

        public static GoalBehavior trapezoidal(final double velocity, final double acceleration) {
            return new GoalBehavior(ProfileType.TRAPEZOIDAL, velocity, acceleration);
        }
    }

    private final Constants.RobotMode mode;
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final IntakePivotIO intakePivotIO;
    private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

    private State desiredGoal = Goal.STOW;
    private double positionSetpointRots;
    private boolean zeroed = false;

    public final LoggedTrigger atSetpoint = group.t(
            "AtSetpoint",
            () -> MathUtil.isNear(positionSetpointRots, inputs.positionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, inputs.velocityRotsPerSec, VelocityToleranceRotsPerSec)
    );

    public IntakePivot(final Constants.RobotMode mode, final HardwareConstants.IntakePivotConstants constants) {
        this.mode = mode;
        this.intakePivotIO = switch (mode) {
            case REAL -> new IntakePivotIOReal(constants);
            case SIM -> new IntakePivotIOSim(constants);
            case REPLAY, DISABLED -> new IntakePivotIO() {};
        };
    }

    @Override
    public void periodic() {
        final double intakePivotPeriodicUpdateStart = Timer.getFPGATimestamp();

        intakePivotIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        final State currentState;
        if (!zeroed) {
            currentState = Dynamic.ZEROING;
            intakePivotIO.toVoltage(ZeroingVolts);

            if (inputs.statorCurrentAmps >= ZeroingCurrentAmps
                    || mode == Constants.RobotMode.SIM) {
                intakePivotIO.setPosition(ZeroPositionRots);
                zeroed = true;

                setPositionImpl(positionSetpointRots);
            }
        } else if (atSetpoint()) {
            currentState = desiredGoal;
        } else {
            currentState = Dynamic.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.name());
        Logger.recordOutput(LogKey + "/CurrentGoal", currentState.name());
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);
        Logger.recordOutput(LogKey + "/Zeroed", zeroed);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakePivotPeriodicUpdateStart)
        );
    }

    public boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    private boolean atGoal(final State goal) {
        return desiredGoal == goal && atSetpoint();
    }

    public LoggedTrigger atGoal(final Goal goal) {
        return group.t("AtGoal/" + goal.name(), () -> atGoal((State)goal));
    }

    private void setPositionImpl(final double positionRots) {
        positionSetpointRots = positionRots;
        if (zeroed) {
            intakePivotIO.toPosition(positionSetpointRots);
        }
    }

    private void setProfiledPositionImpl(
            final double positionRots,
            final double maxVelocityRotsPerSec,
            final double maxAccelRotsPerSecSq
    ) {
        positionSetpointRots = positionRots;
        if (zeroed) {
            intakePivotIO.toProfiledPosition(
                    positionRots,
                    maxVelocityRotsPerSec,
                    maxAccelRotsPerSecSq
            );
        }
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = goal;
        switch (goal.behavior.type) {
            case NONE -> setPositionImpl(goal.positionRots);
            case TRAPEZOIDAL -> setProfiledPositionImpl(
                    goal.positionRots,
                    goal.behavior.velocity,
                    goal.behavior.acceleration
            );
        }
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

    public Command runGoal(final Goal goal) {
        return startIdle(() -> setGoalImpl(goal));
    }

    public Pose3d[] getComponentPoses() {
        final Pose3d pivotPose = new Pose3d(
                SimConstants.IntakePivot.OriginOffset,
                new Rotation3d(
                        0,
                        SimConstants.IntakePivot.ZeroedPositionToSimZero.getRadians()
                                + Units.rotationsToRadians(inputs.positionRots),
                        0
                )
        );

        return new Pose3d[] {
                pivotPose,
                Pose3d.kZero
        };
    }
}
