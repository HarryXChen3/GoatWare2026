package frc.robot.utils.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.constants.SimConstants;

import java.util.function.Supplier;

public class Phoenix6Utils {
    public static final int MaxRetryAttempts = 2;

    /**
     * Performs latency compensation on a refreshed {@link StatusSignal}
     * (using {@link BaseStatusSignal#getLatencyCompensatedValue(StatusSignal, StatusSignal)})
     * <p>
     * Only compensates if the {@link StatusCode} of the {@link StatusSignal}
     * is OK ({@link StatusCode#isOK()})
     * <p>
     * If the {@link StatusCode} is not OK, then it just returns the signal value without latency compensation
     *
     * @param refreshedSignal      the {@link StatusSignal}, ex. Position
     * @param refreshedDeltaSignal the delta/derivative of the {@link StatusSignal}, ex. Velocity
     * @return the latency compensated value
     * @see StatusSignal
     * @see StatusCode
     * @see BaseStatusSignal#getLatencyCompensatedValue(StatusSignal, StatusSignal)
     */
    public static
    <U extends Unit, U_PER_SEC extends PerUnit<U, TimeUnit>, MEAS extends Measure<U>, MEAS_PER_SEC extends Measure<U_PER_SEC>>
    double latencyCompensateIfSignalIsGood(
            final StatusSignal<MEAS> refreshedSignal,
            final StatusSignal<MEAS_PER_SEC> refreshedDeltaSignal
    ) {
        if (refreshedSignal.getStatus().isOK() && refreshedDeltaSignal.getStatus().isOK()) {
            return BaseStatusSignal.getLatencyCompensatedValue(
                    refreshedSignal,
                    refreshedDeltaSignal
            ).magnitude();
        } else {
            return refreshedSignal.getValueAsDouble();
        }
    }

    /**
     * Configures the soft limits on a {@link TalonFX}
     * @param talonFX the {@link TalonFX} to configure
     * @param reverseSoftLimitRots the reverse soft limit, in Rots
     * @param forwardSoftLimitRots the forward soft limit, in Rots
     * @return the {@link StatusCode} after applying
     */
    @SuppressWarnings("UnusedReturnValue")
    public static StatusCode configureTalonFXSoftLimits(
            final TalonFX talonFX,
            final double reverseSoftLimitRots,
            final double forwardSoftLimitRots
    ) {
        final TalonFXConfigurator configurator = talonFX.getConfigurator();
        final TalonFXConfiguration configuration = new TalonFXConfiguration();

        if (Constants.CURRENT_MODE == Constants.RobotMode.REAL) {
            Phoenix6Utils.reportIfNotOk(talonFX, configurator.refresh(configuration));
        } else {
            // use longer timeout when in sim
            Phoenix6Utils.reportIfNotOk(
                    talonFX,
                    configurator.refresh(configuration, SimConstants.CTRE.ConfigTimeoutSeconds)
            );
        }

        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimitRots;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimitRots;
        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        final StatusCode statusCode = tryUntilOk(talonFX, () -> configurator.apply(configuration));
        Phoenix6Utils.reportIfNotOk(talonFX, statusCode);
        return statusCode;
    }

    /**
     * Attempts to invoke the callback until the returned {@link StatusCode#isOK()}
     * @param device the {@link ParentDevice} associated with the callback
     * @param maxAttempts the maximum number of attempts
     * @param apply the callback
     * @return the last {@link StatusCode}
     */
    public static StatusCode tryUntilOk(
            final ParentDevice device,
            final int maxAttempts,
            final Supplier<StatusCode> apply
    ) {
        StatusCode status = StatusCode.OK;
        for (int i = 0; i < maxAttempts; i++) {
            status = apply.get();
            if (status.isOK()) {
                return status;
            } else if (i < maxAttempts - 1) {
                DriverStation.reportWarning(String.format(
                        "Attempt %d on device %d: %s",
                        i + 2, device.getDeviceID(), status.getName()
                ), true);
            }
        }

        reportIfNotOk(device, status);
        return status;
    }

    /**
     * Attempts to invoke the callback until the returned {@link StatusCode#isOK()}
     * @param device the {@link ParentDevice} associated with the callback
     * @param apply the callback
     * @return the last {@link StatusCode}
     */
    public static StatusCode tryUntilOk(final ParentDevice device, final Supplier<StatusCode> apply) {
        return tryUntilOk(device, MaxRetryAttempts, apply);
    }

    /**
     * Reports a status code failure if {@link StatusCode#isOK()} is false.
     * @param device the {@link ParentDevice} that generated the {@link StatusCode}
     * @param statusCode the {@link StatusCode}
     */
    public static void reportIfNotOk(final ParentDevice device, final StatusCode statusCode) {
        if (!statusCode.isOK()) {
            DriverStation.reportError(
                    String.format(
                            "Failed on device %d: %s",
                            device.getDeviceID(),
                            statusCode.getName()
                    ),
                    true
            );
        }
    }

    /**
     * Assert that a {@link StatusCode} must be {@link StatusCode#isOK()}.
     *
     * @param statusCode the {@link StatusCode}
     * @throws StatusCodeAssertionException if the {@link StatusCode} is not {@link StatusCode#isOK()}
     */
    public static void assertIsOK(final StatusCode statusCode) {
        if (!statusCode.isOK()) {
            throw new StatusCodeAssertionException(statusCode);
        }
    }

    /**
     * Exception thrown when a {@link StatusCode} assertion fails.
     *
     * @see Phoenix6Utils#assertIsOK(StatusCode)
     */
    public static class StatusCodeAssertionException extends RuntimeException {
        public StatusCodeAssertionException(final StatusCode expected, final StatusCode got) {
            super(String.format("Expected StatusCode %s; got %s", expected.getName(), got.getName()));
        }

        public StatusCodeAssertionException(final StatusCode got) {
            this(StatusCode.OK, got);
        }
    }
}