package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class ShooterIOReal implements ShooterIO {
    private final HardwareConstants.ShooterConstants constants;
    private final TalonFXS masterMotor;
    private final TalonFXS followerMotor;

    private final VelocityVoltage velocityVoltage;
    private final VoltageOut voltageOut;
    private final Follower follower;

    private final StatusSignal<Angle> masterPosition;
    private final StatusSignal<AngularVelocity> masterVelocity;
    private final StatusSignal<Voltage> masterVoltage;
    private final StatusSignal<Current> masterTorqueCurrent;
    private final StatusSignal<Temperature> masterDeviceTemp;

    private final StatusSignal<Angle> followerPosition;
    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerVoltage;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Temperature> followerDeviceTemp;

    public ShooterIOReal(final HardwareConstants.ShooterConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        this.masterMotor = new TalonFXS(constants.masterId(), bus.p6Bus);
        this.followerMotor = new TalonFXS(constants.followerId(), bus.p6Bus);

        this.velocityVoltage = new VelocityVoltage(0);
        this.voltageOut = new VoltageOut(0);
        this.follower = new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        this.masterPosition = masterMotor.getPosition(false);
        this.masterVelocity = masterMotor.getVelocity(false);
        this.masterVoltage = masterMotor.getMotorVoltage(false);
        this.masterTorqueCurrent = masterMotor.getTorqueCurrent(false);
        this.masterDeviceTemp = masterMotor.getDeviceTemp(false);

        this.followerPosition = followerMotor.getPosition(false);
        this.followerVelocity = followerMotor.getVelocity(false);
        this.followerVoltage = followerMotor.getMotorVoltage(false);
        this.followerTorqueCurrent = followerMotor.getTorqueCurrent(false);
        this.followerDeviceTemp = followerMotor.getDeviceTemp(false);

        RefreshAll.add(
                bus,
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                masterDeviceTemp,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent,
                followerDeviceTemp
        );

        config();
    }

    @Override
    public void updateInputs(final ShooterIO.ShooterIOInputs inputs) {
        inputs.masterPositionRots = masterPosition.getValueAsDouble();
        inputs.masterVelocityRotsPerSec = masterVelocity.getValueAsDouble();
        inputs.masterVoltage = masterVoltage.getValueAsDouble();
        inputs.masterTorqueCurrentAmps = masterTorqueCurrent.getValueAsDouble();
        inputs.masterTempCelsius = masterDeviceTemp.getValueAsDouble();

        inputs.followerPositionRots = followerPosition.getValueAsDouble();
        inputs.followerVelocityRotsPerSec = followerVelocity.getValueAsDouble();
        inputs.followerVoltage = followerVoltage.getValueAsDouble();
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
        inputs.followerTempCelsius = followerDeviceTemp.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXSConfiguration rightMotorConfiguration = new TalonFXSConfiguration();
        rightMotorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(0.121)
                .withKA(0)
                .withKP(0.5)
                .withKD(0);
        rightMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
        rightMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0;
        rightMotorConfiguration.CurrentLimits.StatorCurrentLimit = 42;
        rightMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotorConfiguration.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        rightMotorConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        rightMotorConfiguration.ExternalFeedback.SensorToMechanismRatio = constants.gearing();
        rightMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfiguration.MotorOutput.PeakReverseDutyCycle = 0;
        rightMotorConfiguration.Voltage.PeakForwardVoltage = 16;
        rightMotorConfiguration.Voltage.PeakReverseVoltage = 0;

        final TalonFXSConfiguration leftMotorConfiguration = rightMotorConfiguration.clone();
        leftMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotorConfiguration.MotorOutput.PeakForwardDutyCycle = 0;
        leftMotorConfiguration.MotorOutput.PeakReverseDutyCycle = -1;

        Phoenix6Utils.tryUntilOk(masterMotor, () -> masterMotor.getConfigurator().apply(leftMotorConfiguration));
        Phoenix6Utils.tryUntilOk(followerMotor, () -> followerMotor.getConfigurator().apply(rightMotorConfiguration));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                masterDeviceTemp,
                followerDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                masterMotor,
                followerMotor
        );
    }

    @Override
    public void toShooterVelocity(final double shooterVelocityRotsPerSec) {
        masterMotor.setControl(velocityVoltage.withVelocity(shooterVelocityRotsPerSec));
        followerMotor.setControl(follower);
    }

    @Override
    public void toShooterVoltage(final double shooterVolts) {
        masterMotor.setControl(voltageOut.withOutput(shooterVolts));
        followerMotor.setControl(follower);
    }
}
