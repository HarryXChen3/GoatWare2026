package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class TurretIOReal implements TurretIO {
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX motor;
    private final CANcoder CANcoder;

    private final PositionVoltage positionVoltage;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Angle> motorRotorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    private final StatusSignal<Angle> CANcoderPosition;

    private boolean positionSeeded = false;

    public TurretIOReal(final HardwareConstants.TurretConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), bus.p6Bus);
        this.CANcoder = new CANcoder(constants.CANcoderId(), bus.p6Bus);

        this.positionVoltage = new PositionVoltage(0);
        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.motorPosition = motor.getPosition(false);
        this.motorRotorPosition = motor.getRotorPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        this.CANcoderPosition = CANcoder.getPosition(false);

        RefreshAll.add(
                bus,
                motorPosition,
                motorRotorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp,
                CANcoderPosition
        );

        config();
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.motorPositionRots = motorPosition.getValueAsDouble();
        inputs.motorRotorPositionRots = motorRotorPosition.getValueAsDouble();
        inputs.motorVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorDeviceTemp.getValueAsDouble();

        inputs.CANcoderPositionRots = CANcoderPosition.getValueAsDouble();

        inputs.motorConnected = motor.isConnected();
        inputs.CANcoderConnected = CANcoder.isConnected();
        inputs.positionSeeded = positionSeeded;
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKP(250)
                .withKD(0.1);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.MotionMagic.MotionMagicExpo_kV = 0.12;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 100;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.motorToTurretGearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfiguration.Voltage.PeakForwardVoltage = 6;
        motorConfiguration.Voltage.PeakReverseVoltage = -6;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorRotorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                CANcoderPosition
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                motorDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor,
                CANcoder
        );
    }

    @Override
    public void seedTurretPosition(final Rotation2d turretPosition) {
        if (positionSeeded) {
            DriverStation.reportWarning(
                    "Attempted to seed turret position more than once! This is a bug.",
                    true
            );
            return;
        }

        final double turretPositionRots = turretPosition.getRotations();
        final StatusCode statusCode =
                Phoenix6Utils.tryUntilOk(motor, 10, () -> motor.setPosition(turretPositionRots));
        if (statusCode.isOK()) {
            positionSeeded = true;
        }
    }

    @Override
    public void trackTurretPosition(final double turretPositionRots, final double turretVelocityRotsPerSec) {
        motor.setControl(positionVoltage
                .withPosition(turretPositionRots)
                .withVelocity(turretVelocityRotsPerSec));
    }

    @Override
    public void toTurretPosition(final double turretPositionRots) {
        motor.setControl(motionMagicExpoVoltage.withPosition(turretPositionRots));
    }

    @Override
    public void toTurretVoltage(final double turretVolts) {
        motor.setControl(voltageOut.withOutput(turretVolts));
    }
}
