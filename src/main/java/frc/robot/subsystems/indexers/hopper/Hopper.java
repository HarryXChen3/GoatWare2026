package frc.robot.subsystems.indexers.hopper;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    protected static final String LogKey = "Hopper";

    public enum Goal {
        OFF(0),
        FEED(10);

        public final double volts;

        Goal(final double volts) {
            this.volts = volts;
        }
    }

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.OFF;

    public Hopper(final Constants.RobotMode mode, final HardwareConstants.HopperConstants constants) {
        this.hopperIO = switch (mode) {
            case REAL -> new HopperIOReal(constants);
            case SIM -> new HopperIOSim(constants);
            case REPLAY, DISABLED -> new HopperIO() {};
        };

        this.inputs = new HopperIOInputsAutoLogged();

        this.hopperIO.config();
    }

    @Override
    public void periodic() {
        final double hopperPeriodicUpdateStart = Timer.getFPGATimestamp();

        hopperIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - hopperPeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> {
                    desiredGoal = goal;
                    hopperIO.toHopperVoltage(goal.volts);
                },
                () -> desiredGoal = Goal.OFF
        );
    }
}
