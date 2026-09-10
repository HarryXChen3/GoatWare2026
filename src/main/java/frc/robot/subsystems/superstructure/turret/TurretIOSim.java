package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimFeedbackSensor;
import frc.robot.utils.sim.motors.TalonFXSim;

public class TurretIOSim implements TurretIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX motor;
    private final CANcoder CANcoder;

    private final DCMotorSim dcMotorSim;
    private final Rotation2d initialRandPosition;

    private final TalonFXSim motorSim;
    private final SimTurretCANcoder simCANcoder;

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

    public TurretIOSim(final HardwareConstants.TurretConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), bus.p6Bus);
        this.CANcoder = new CANcoder(constants.CANcoderId(), bus.p6Bus);

        this.initialRandPosition = Rotation2d.fromRotations(
                Math.random()
                        * (constants.forwardLimitRots() - constants.reverseLimitRots())
                        + constants.reverseLimitRots()
        );

        final double motorToTurretGearing = constants.motorToTurretGearing();
        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        this.dcMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        dcMotor,
                        0.0977,
                        motorToTurretGearing
                ),
                dcMotor
        );
        this.dcMotorSim.setState(initialRandPosition.getRadians(), 0);

        this.motorSim = new TalonFXSim(
                motor,
                motorToTurretGearing,
                dcMotorSim::update,
                voltage -> dcMotorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                dcMotorSim::getAngularPositionRad,
                dcMotorSim::getAngularVelocityRadPerSec
        );
        this.simCANcoder = new SimTurretCANcoder(constants, CANcoder);

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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            motorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                motor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
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
                .withKS(0.25)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(5.248)
                .withKA(0.0088)
                .withKP(103.385)
                .withKD(5.169);
        motorConfiguration.Slot1 = new Slot1Configs()
                .withKS(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKV(0)
                .withKA(0)
                .withKP(80)
                .withKD(4);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.MotionMagic.MotionMagicExpo_kV = 0.12;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.motorToTurretGearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfiguration));

        final CANcoderConfiguration CANcoderConfiguration = new CANcoderConfiguration();
        CANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        CANcoderConfiguration.MagnetSensor.MagnetOffset = 0;
        Phoenix6Utils.tryUntilOk(
                CANcoder,
                () -> CANcoder.getConfigurator().apply(CANcoderConfiguration)
        );

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

        final TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        motorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        final CANcoderSimState CANcoderSimState = CANcoder.getSimState();
        CANcoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        CANcoderSimState.SensorOffset = 0;

        simCANcoder.setRawPosition(initialRandPosition.getRotations());
        motorSim.attachFeedbackSensor(simCANcoder);
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
                .withSlot(0)
                .withPosition(turretPositionRots)
                .withVelocity(turretVelocityRotsPerSec)
        );
    }

    @Override
    public void toTurretPosition(final double turretPositionRots) {
        motor.setControl(motionMagicExpoVoltage
                .withSlot(1)
                .withPosition(turretPositionRots));
    }

    @Override
    public void toTurretVoltage(final double turretVolts) {
        motor.setControl(voltageOut.withOutput(turretVolts));
    }

    private static class SimTurretCANcoder implements SimFeedbackSensor {
        private final HardwareConstants.TurretConstants constants;

        private final CANcoder CANcoder;

        private final CANcoderSimState simState;

        public SimTurretCANcoder(
                final HardwareConstants.TurretConstants constants,
                final CANcoder CANcoder
        ) {
            this.constants = constants;
            this.CANcoder = CANcoder;

            this.simState = CANcoder.getSimState();
        }

        @Override
        public void setSupplyVoltage(final double volts) {
            Phoenix6Utils.reportIfNotOk(CANcoder, simState.setSupplyVoltage(volts));
        }

        @Override
        public void setRawPosition(final double rotations) {
            Phoenix6Utils.reportIfNotOk(
                    CANcoder,
                    simState.setRawPosition(rotations * constants.CANcoderGearing())
            );
        }

        @Override
        public void addPosition(final double deltaRotations) {
            Phoenix6Utils.reportIfNotOk(
                    CANcoder,
                    simState.addPosition(deltaRotations * constants.CANcoderGearing())
            );
        }

        @Override
        public void setVelocity(final double rotationsPerSec) {
            Phoenix6Utils.reportIfNotOk(
                    CANcoder,
                    simState.setVelocity(rotationsPerSec * constants.CANcoderGearing())
            );
        }
    }
}
