package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class StaticShot implements ShotProvider<ShotProvider.Kind.Static> {
    public static Rotation2d angleToTarget(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final Pose2d targetPose
    ) {
        return targetPose.getTranslation()
                .minus(turretTranslation)
                .getAngle()
                .minus(robotPose.getRotation());
    }

    @Override
    public ShotParameters getParameters(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final ChassisSpeeds robotRelativeSpeeds,
            final Pose2d targetPose
    ) {
        return new ShotParameters(
                ShotParameters.getShot(
                        turretTranslation
                                .getDistance(targetPose.getTranslation())
                ),
                angleToTarget(robotPose, turretTranslation, targetPose),
                Units.radiansToRotations(-robotRelativeSpeeds.omegaRadiansPerSecond)
        );
    }
}
