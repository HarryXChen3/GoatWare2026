package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;

import java.util.function.Function;
import java.util.function.Supplier;

public class StaticShot {
    public static final InterpolatingTreeMap<Double, ShotParameters.Shooter> ShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters.Shooter::interpolate);
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

    public static Rotation2d angleToTarget(final Pose2d robotPose, final Pose2d turretPose, final Pose2d targetPose) {
        return targetPose.getTranslation()
                .minus(turretPose.getTranslation())
                .getAngle()
                .minus(robotPose.getRotation());
    }

    public static ShotParameters getParameters(
            final Pose2d robotPose,
            final Pose2d turretPose,
            final ChassisSpeeds robotRelativeSpeeds,
            final Pose2d targetPose
    ) {
        return new ShotParameters(
                ShotMap.get(
                        turretPose
                                .getTranslation()
                                .getDistance(targetPose.getTranslation())
                ),
                angleToTarget(robotPose, turretPose, targetPose),
                Units.radiansToRotations(-robotRelativeSpeeds.omegaRadiansPerSecond)
        );
    }

    public static Supplier<ShotParameters> parametersSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Function<Pose2d, Pose2d> toTurretPoseFn,
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> {
            final Pose2d robotPose = robotPoseSupplier.get();
            return StaticShot.getParameters(
                    robotPose,
                    toTurretPoseFn.apply(robotPose),
                    robotRelativeSpeedsSupplier.get(),
                    targetPoseSupplier.get()
            );
        };
    }
}
