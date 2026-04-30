package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class StaticShot implements ShotProvider<ShotProvider.Kind.Static> {
    public static Rotation2d angleToTarget(
            final Translation2d from,
            final Pose2d target
    ) {
        return target.getTranslation()
                .minus(from)
                .getAngle();
    }

    public static Rotation2d anglePointingShooter(
            final Pose2d robotPose,
            final Transform2d robotToShooter,
            final Pose2d targetPose
    ) {
        final Translation2d robotTranslation = robotPose.getTranslation();
        final double distanceToRobot = robotTranslation
                .getDistance(targetPose.getTranslation());
        final Rotation2d toTarget = angleToTarget(robotTranslation, targetPose);

        final Rotation2d offsetAngle = Rotation2d.fromRadians(
                Math.asin(MathUtil.clamp(
                        robotToShooter.getTranslation().getY() / distanceToRobot,
                        -1.0,
                        1.0
                ))
        );

        return toTarget
                .plus(offsetAngle)
                .plus(robotToShooter.getRotation());
    }

    @Override
    public ShotParameters getParameters(
            final Pose2d robotPose,
            final Transform2d robotToShooter,
            final ChassisSpeeds robotRelativeSpeeds,
            final Pose2d targetPose
    ) {
        final Translation2d turretTranslation = robotPose
                .transformBy(robotToShooter)
                .getTranslation();
        return new ShotParameters(
                ShotParameters.getShot(
                        turretTranslation
                                .getDistance(targetPose.getTranslation())
                ),
                anglePointingShooter(robotPose, robotToShooter, targetPose),
                0
        );
    }
}
