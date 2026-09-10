package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class IntakePivotIOReal implements IntakePivotIO {
    private final HardwareConstants.IntakePivotConstants constants;
    private final TalonFX motor;

    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage;
    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temperature;

    public IntakePivotIOReal(final HardwareConstants.IntakePivotConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), bus.p6Bus);

        this.dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0, 0, 0);
        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.position = motor.getPosition(false);
        this.velocity = motor.getVelocity(false);
        this.voltage = motor.getMotorVoltage(false);
        this.statorCurrent = motor.getStatorCurrent(false);
        this.temperature = motor.getDeviceTemp(false);

        RefreshAll.add(
                bus,
                position,
                velocity,
                voltage,
                statorCurrent,
                temperature
        );

        config();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKP(640)
                .withKD(4.5)
                .withKS(0)
                .withKG(-0.5)
                .withGravityType(GravityTypeValue.Arm_Cosine);
        motorConfiguration.Slot1 = new Slot1Configs()
                .withKP(0.5)
                .withKG(-0.5);
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.Voltage.PeakForwardVoltage = 4;
        motorConfiguration.Voltage.PeakReverseVoltage = -4;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                position,
                velocity,
                voltage,
                statorCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                temperature
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor
        );
    }

    @Override
    public void updateInputs(final IntakePivotIOInputs inputs) {
        inputs.positionRots = position.getValueAsDouble();
        inputs.velocityRotsPerSec = velocity.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.tempCelsius = temperature.getValueAsDouble();
        inputs.connected = motor.isConnected();
    }

    @Override
    public void toPosition(final double positionRots) {
        motor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void toProfiledPosition(
            final double positionRots,
            final double maxVelocityRotsPerSec,
            final double maxAccelRotsPerSecSq
    ) {
        motor.setControl(dynamicMotionMagicVoltage
                .withPosition(positionRots)
                .withVelocity(maxVelocityRotsPerSec)
                .withAcceleration(maxAccelRotsPerSecSq)
        );
    }

    @Override
    public void toVoltage(final double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setPosition(final double positionRots) {
        Phoenix6Utils.tryUntilOk(motor, () -> motor.setPosition(positionRots));
    }
}
