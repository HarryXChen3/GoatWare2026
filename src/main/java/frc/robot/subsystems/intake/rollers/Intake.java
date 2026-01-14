package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    protected static final String LogKey = "Shooter";
    private static final double VelocityToleranceRotsPerSec = 0.5;

    public enum Goal {
        OFF(0),
        INTAKING(10);

        public final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.OFF;

    public Intake(final Constants.RobotMode mode, final HardwareConstants.IntakeConstants constants) {
        this.intakeIO = switch (mode) {
            case REAL -> new IntakeIOReal(constants);
            case SIM -> new IntakeIOSim(constants);
            case REPLAY, DISABLED -> new IntakeIO() {};
        };

        this.inputs = new IntakeIOInputsAutoLogged();

        this.intakeIO.config();
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakePeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> {
                    desiredGoal = goal;
                    intakeIO.toIntakeVelocity(goal.velocityRotsPerSec);
                },
                () -> desiredGoal = Goal.OFF
        );
    }
}
