package frc.robot.subsystems.indexer.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class HopperIOReal implements HopperIO {
    private final HardwareConstants.HopperConstants constants;
    private final TalonFX motor1;
    private final TalonFX motor2;

    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final Follower follower;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public HopperIOReal(final HardwareConstants.HopperConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        this.motor1 = new TalonFX(constants.motor1Id(), bus.p6Bus);
        this.motor2 = new TalonFX(constants.motor2Id(), bus.p6Bus);

        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.follower = new Follower(motor1.getDeviceID(), MotorAlignmentValue.Aligned);

        this.motorPosition = motor1.getPosition(false);
        this.motorVelocity = motor1.getVelocity(false);
        this.motorVoltage = motor1.getMotorVoltage(false);
        this.motorTorqueCurrent = motor1.getTorqueCurrent(false);
        this.motorDeviceTemp = motor1.getDeviceTemp(false);

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
                .withKV(1.1)
                .withKA(0)
                .withKP(4)
                .withKD(0);
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.Voltage.PeakForwardVoltage = 16;
        motorConfiguration.Voltage.PeakReverseVoltage = 0;
        Phoenix6Utils.tryUntilOk(motor1, () -> motor1.getConfigurator().apply(motorConfiguration));
        Phoenix6Utils.tryUntilOk(motor2, () -> motor2.getConfigurator().apply(motorConfiguration));

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
                motor1,
                motor2
        );
    }

    @Override
    public void toHopperTorqueCurrent(final double hopperTorqueCurrentAmps) {
        motor1.setControl(torqueCurrentFOC.withOutput(hopperTorqueCurrentAmps));
        motor2.setControl(follower);
    }

    @Override
    public void toHopperVoltage(final double hopperVolts) {
        motor1.setControl(voltageOut.withOutput(hopperVolts));
        motor2.setControl(follower);
    }
}
