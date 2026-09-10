package frc.robot.subsystems.indexer.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class FeederIOReal implements FeederIO {
    private final HardwareConstants.FeederConstants constants;
    private final TalonFX motor;

    private final VelocityVoltage velocityVoltage;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public FeederIOReal(final HardwareConstants.FeederConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), bus.p6Bus);

        this.velocityVoltage = new VelocityVoltage(0);
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

        config();
    }

    @Override
    public void updateInputs(final FeederIOInputs inputs) {
        inputs.rollerPositionRots = motorPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.rollerVoltage = motorVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = motorDeviceTemp.getValueAsDouble();

        inputs.tofDetected = false;
    }

    @Override
    public void config() {
        final TalonFXConfiguration feederConfiguration = new TalonFXConfiguration();
        feederConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(0.2800000011920929)
                .withKA(0)
                .withKP(160)
                .withKD(0);
        feederConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        feederConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfiguration.CurrentLimits.StatorCurrentLimit = 120;
        feederConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        feederConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        feederConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(feederConfiguration));

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
    public void toFeederVelocity(final double feederVelocityRotsPerSec) {
        motor.setControl(velocityVoltage.withVelocity(feederVelocityRotsPerSec));
    }

    @Override
    public void toFeederVoltage(final double feederVolts) {
        motor.setControl(voltageOut.withOutput(feederVolts));
    }
}
