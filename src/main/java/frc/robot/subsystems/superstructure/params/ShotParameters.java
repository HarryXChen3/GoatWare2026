package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Container;

import java.util.function.Supplier;

public record ShotParameters(
        Shooter shooter,
        Rotation2d robotAngle,
        double robotOmegaRadsPerSec
) {
    private static final InterpolatingTreeMap<Double, Shooter> ShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Shooter::interpolate);
    static {
        ShotMap.put(1.3, new ShotParameters.Shooter(21, 0));
        ShotMap.put(1.5, new ShotParameters.Shooter(21, 0.008));
        ShotMap.put(2.0, new ShotParameters.Shooter(21, 0.0175));
        ShotMap.put(2.5, new ShotParameters.Shooter(21, 0.031));
        ShotMap.put(3.0, new ShotParameters.Shooter(22.5, 0.034));
        ShotMap.put(3.5, new ShotParameters.Shooter(23, 0.045));
        ShotMap.put(4.0, new ShotParameters.Shooter(24, 0.0475));
        ShotMap.put(4.5, new ShotParameters.Shooter(25, 0.05));
        ShotMap.put(5.0, new ShotParameters.Shooter(26, 0.0525));
        ShotMap.put(5.5, new ShotParameters.Shooter(27, 0.055));
        ShotMap.put(6.0, new ShotParameters.Shooter(27.75, 0.0565));
    }

    private static final InterpolatingDoubleTreeMap TimeOfFlightMap = new InterpolatingDoubleTreeMap();
    static {
        TimeOfFlightMap.put(1.31, 1.06);
        TimeOfFlightMap.put(2.17, 1.0);
        TimeOfFlightMap.put(2.72, 1.02);
        TimeOfFlightMap.put(3.51, 1.06);
        TimeOfFlightMap.put(4.63, 1.2);
        TimeOfFlightMap.put(4.9, 1.24);
        TimeOfFlightMap.put(5.73, 1.31);
    }

    public static Shooter getShot(final double distanceMeters) {
        return ShotMap.get(distanceMeters);
    }

    public static double getTimeOfFlight(final double distanceMeters) {
        return TimeOfFlightMap.get(distanceMeters);
    }

    public record Shooter(
            double shooterVelocityRotsPerSec,
            double hoodPositionRots
    ) implements Interpolatable<Shooter> {
        @Override
        public Shooter interpolate(final Shooter endValue, final double t) {
            return new Shooter(
                    MathUtil.interpolate(
                            shooterVelocityRotsPerSec,
                            endValue.shooterVelocityRotsPerSec,
                            t
                    ),
                    MathUtil.interpolate(
                            hoodPositionRots,
                            endValue.hoodPositionRots,
                            t
                    )
            );
        }
    }

    public record CachedShot(Supplier<ShotParameters> shot, Command clear) implements Supplier<ShotParameters> {
        @Override
        public ShotParameters get() {
            return shot.get();
        }
    }

    public static CachedShot getCached(final Supplier<ShotParameters> shot) {
        final Container<ShotParameters> parameters = Container.empty();
        final Supplier<ShotParameters> cached = () -> {
            if (parameters.hasValue()) {
                return parameters.get();
            }

            final ShotParameters params = shot.get();
            parameters.set(params);
            return params;
        };

        return new CachedShot(cached, Commands.run(parameters::clear));
    }
}
