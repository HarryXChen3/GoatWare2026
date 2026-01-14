package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public record ShotParameters(
        double shooterVelocityRotsPerSec,
        double hoodPositionRots
) implements Interpolatable<ShotParameters> {
    @Override
    public ShotParameters interpolate(final ShotParameters endValue, final double t) {
        return new ShotParameters(
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
