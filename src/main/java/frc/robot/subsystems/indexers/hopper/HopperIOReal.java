package frc.robot.subsystems.indexers.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class HopperIOReal implements HopperIO {
    private final HardwareConstants.HopperConstants constants;
    private final TalonFX motor;

    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public HopperIOReal(final HardwareConstants.HopperConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorId(), p6Bus);

        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        RefreshAll.add(
                bus,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp
        );
    }

    @Override
    public void updateInputs(final HopperIOInputs inputs) {
        inputs.hopperPositionRots = motorPosition.getValueAsDouble();
        inputs.hopperVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.hopperVoltage = motorVoltage.getValueAsDouble();
        inputs.hopperTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.hopperTempCelsius = motorDeviceTemp.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(0)
                .withKA(0)
                .withKP(120)
                .withKD(40);
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                motorDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor
        );
    }

    @Override
    public void toHopperTorqueCurrent(final double hopperTorqueCurrentAmps) {
        motor.setControl(torqueCurrentFOC.withOutput(hopperTorqueCurrentAmps));
    }

    @Override
    public void toHopperVoltage(final double hopperVolts) {
        motor.setControl(voltageOut.withOutput(hopperVolts));
    }
}
