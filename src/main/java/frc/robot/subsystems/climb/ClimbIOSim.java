package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.motors.TalonFXSim;

public class ClimbIOSim implements ClimbIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ClimbConstants constants;

    private final TalonFX motor;
    private final TalonFXSim motorSim;

    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    public ClimbIOSim(final HardwareConstants.ClimbConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), bus.p6Bus);

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        final ElevatorSim elevatorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        dcMotor,
                        SimConstants.Climb.ClimbWeightKgs,
                        SimConstants.Climb.PulleyRadiusMeters,
                        constants.gearing()
                ),
                dcMotor,
                constants.lowerLimitRots() * SimConstants.Climb.PulleyCircumferenceMeters,
                constants.upperLimitRots() * SimConstants.Climb.PulleyCircumferenceMeters,
                false,
                0
        );
        this.motorSim = new TalonFXSim(
                motor,
                constants.gearing(),
                elevatorSim::update,
                voltage -> elevatorSim.setInputVoltage(SimUtils.addMotorFriction(voltage, 0.25)),
                () -> Units.rotationsToRadians(elevatorSim.getPositionMeters() / SimConstants.Climb.PulleyCircumferenceMeters),
                () -> Units.rotationsToRadians(elevatorSim.getVelocityMetersPerSecond() / SimConstants.Climb.PulleyCircumferenceMeters)
        );

        this.positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
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
    public void updateInputs(final ClimbIOInputs inputs) {
        inputs.motorPositionRots = motorPosition.getValueAsDouble();
        inputs.motorVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorDeviceTemp.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(0)
                .withKA(0)
                .withKP(40)
                .withKD(4);
        motorConfiguration.Slot1 = new Slot1Configs()
                .withKS(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(0)
                .withKA(0)
                .withKP(260)
                .withKD(12);
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 120;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -120;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 120;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
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

        final TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        motorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void deployToPosition(final double deployPositionRots) {
        motor.setControl(positionTorqueCurrentFOC
                .withPosition(deployPositionRots)
                .withSlot(0));
    }

    @Override
    public void climbToPosition(final double climbPositionRots) {
        motor.setControl(positionTorqueCurrentFOC
                .withPosition(climbPositionRots)
                .withSlot(1));
    }

    @Override
    public void toClimbTorqueCurrent(final double climbTorqueCurrentAmps) {
        motor.setControl(torqueCurrentFOC.withOutput(climbTorqueCurrentAmps));
    }

    @Override
    public void toClimbVoltage(final double climbVolts) {
        motor.setControl(voltageOut.withOutput(climbVolts));
    }
}
