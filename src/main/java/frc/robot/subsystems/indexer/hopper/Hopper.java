package frc.robot.subsystems.indexer.hopper;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    protected static final String LogKey = "Hopper";

    public enum ControlMode {
        Voltage,
        TorqueCurrent
    }

    public enum Goal {
        OFF(ControlMode.Voltage, 0),
        FEED(ControlMode.TorqueCurrent, 20);

        public final ControlMode controlMode;
        public final double output;

        Goal(final ControlMode controlMode, final double output) {
            this.controlMode = controlMode;
            this.output = output;
        }
    }

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs;

    private ControlMode controlMode;
    private double setpointOutput;
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
        Logger.recordOutput(LogKey + "/ControlMode", controlMode);
        Logger.recordOutput(LogKey + "/SetpointOutput", setpointOutput);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - hopperPeriodicUpdateStart)
        );
    }

    private void setVoltageImpl(final double volts) {
        controlMode = ControlMode.Voltage;
        setpointOutput = volts;
        hopperIO.toHopperVoltage(volts);
    }

    private void setTorqueCurrentImpl(final double torqueCurrentAmps) {
        controlMode = ControlMode.TorqueCurrent;
        setpointOutput = torqueCurrentAmps;
        hopperIO.toHopperTorqueCurrent(torqueCurrentAmps);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = goal;
        switch (goal.controlMode) {
            case Voltage -> setVoltageImpl(goal.output);
            case TorqueCurrent -> setTorqueCurrentImpl(goal.output);
        }
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.OFF)
        );
    }
}
