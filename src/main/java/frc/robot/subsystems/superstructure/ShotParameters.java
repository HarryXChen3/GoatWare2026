package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.Container;

import java.util.function.Supplier;

public record ShotParameters(
        Shooter shooter,
        Rotation2d robotAngle,
        double robotOmegaRadsPerSec
) {
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
