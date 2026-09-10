package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakePivotIOSim implements IntakePivotIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakePivotConstants constants;

    private final TalonFX motor;
    private final TalonFXSim motorSim;

    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage;
    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temperature;

    public IntakePivotIOSim(final HardwareConstants.IntakePivotConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        this.motor = new TalonFX(constants.motorId(), bus.p6Bus);

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);
        final SingleJointedArmSim armSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        dcMotor,
                        0.075,
                        constants.gearing()
                ),
                dcMotor,
                constants.gearing(),
                Units.inchesToMeters(15),
                Units.rotationsToRadians(constants.reverseLimitRots()),
                Units.rotationsToRadians(constants.forwardLimitRots()),
                true,
                Units.rotationsToRadians(-0.3)
        );
        this.motorSim = new TalonFXSim(
                motor,
                constants.gearing(),
                armSim::update,
                armSim::setInputVoltage,
                armSim::getAngleRads,
                armSim::getVelocityRadPerSec
        );

        this.dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0, 0, 0);
        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.position = motor.getPosition(false);
        this.velocity = motor.getVelocity(false);
        this.voltage = motor.getMotorVoltage(false);
        this.current = motor.getStatorCurrent(false);
        this.temperature = motor.getDeviceTemp(false);

        RefreshAll.add(
                bus,
                position,
                velocity,
                voltage,
                current,
                temperature
        );

        config();

        final TalonFXSimState simState = motor.getSimState();
        simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        final Notifier notifier = new Notifier(() -> motorSim.update(deltaTime.get()));
        ToClose.add(notifier);
        notifier.setName("IntakePivotSim");
        notifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKP(640)
                .withKD(4.5)
                .withKG(-0.5)
                .withGravityType(GravityTypeValue.Arm_Cosine);
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfiguration));
    }

    @Override
    public void updateInputs(final IntakePivotIOInputs inputs) {
        inputs.positionRots = position.getValueAsDouble();
        inputs.velocityRotsPerSec = velocity.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
        inputs.statorCurrentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temperature.getValueAsDouble();
        inputs.connected = true;
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
        motor.setPosition(positionRots);
    }
}
