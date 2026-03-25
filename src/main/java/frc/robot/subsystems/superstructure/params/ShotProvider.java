package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;
import java.util.function.Supplier;

public interface ShotProvider<T extends ShotProvider.Kind> {
    interface Kind {
        class Static implements Kind {}
        class Moving implements Kind {}
    }

    ShotParameters getParameters(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final ChassisSpeeds robotSpeeds,
            final Pose2d targetPose
    );

    default Supplier<ShotParameters> parametersSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Function<Pose2d, Translation2d> toTurretFn,
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> {
            final Pose2d robotPose = robotPoseSupplier.get();
            return getParameters(
                    robotPose,
                    toTurretFn.apply(robotPose),
                    robotRelativeSpeedsSupplier.get(),
                    targetPoseSupplier.get()
            );
        };
    }
}
