package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import java.util.function.Supplier;

public class StaticShot {
    private static final InterpolatingTreeMap<Double, ShotParameters> shotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters::interpolate);
    static {
        shotMap.put(0d, new ShotParameters(0, 0));
        shotMap.put(10d, new ShotParameters(40, 0.1));
    }

    public static ShotParameters getParameters(final Pose2d currentPose, final Pose2d targetPose) {
        return shotMap.get(
                currentPose
                        .getTranslation()
                        .getDistance(targetPose.getTranslation())
        );
    }

    public static Supplier<ShotParameters> parametersSupplier(
            final Supplier<Pose2d> currentPoseSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> StaticShot.getParameters(currentPoseSupplier.get(), targetPoseSupplier.get());
    }
}
