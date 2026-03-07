package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

public class MovingTOFShot {
    private static final InterpolatingDoubleTreeMap TimeOfFlightMap = new InterpolatingDoubleTreeMap();
    static {
        TimeOfFlightMap.put(1.31, 1.61);
        TimeOfFlightMap.put(2.17, 1.0);
        TimeOfFlightMap.put(2.72, 1.02);
        TimeOfFlightMap.put(3.51, 1.08);
        TimeOfFlightMap.put(4.63, 1.2);
        TimeOfFlightMap.put(4.9, 1.225);
        TimeOfFlightMap.put(5.73, 1.315);
    }

    public static ShotParameters getParameters(
            final Pose2d robotPose,
            final Pose2d turretPose,
            final ChassisSpeeds robotRelativeSpeeds,
            final Pose2d targetPose
    ) {
        final double distance = turretPose
                .getTranslation()
                .getDistance(targetPose.getTranslation());
        final Rotation2d robotAngle = robotPose.getRotation();
        final ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                robotAngle
        );

        final Transform2d turretOffset = turretPose.minus(robotPose);
        final double robotOmegaRadiansPerSec = fieldRelativeSpeeds.omegaRadiansPerSecond;

        final double tangentVx = robotOmegaRadiansPerSec *
                new Translation2d(turretOffset.getY(), turretOffset.getX())
                        .rotateBy(robotAngle)
                        .getX();
        final double tangentVy = robotOmegaRadiansPerSec *
                turretOffset
                        .getTranslation()
                        .rotateBy(robotAngle)
                        .getX();

        final ChassisSpeeds turretFieldVelocity = new ChassisSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond + tangentVx,
                fieldRelativeSpeeds.vyMetersPerSecond + tangentVy,
                robotOmegaRadiansPerSec
        );

        final Pose2d[] poses = new Pose2d[20];

        double timeOfFlight = TimeOfFlightMap.get(distance);
        Pose2d futureTurretPose = turretPose;
        double futureDistance = distance;

        for (int i = 0; i < 20; i++) {
            final ChassisSpeeds offsetToFuture = turretFieldVelocity.times(timeOfFlight);
            timeOfFlight = TimeOfFlightMap.get(futureDistance);
            futureTurretPose = turretPose.plus(
                    new Transform2d(
                            offsetToFuture.vxMetersPerSecond,
                            offsetToFuture.vyMetersPerSecond,
                            Rotation2d.kZero)
            );
            futureDistance = targetPose.getTranslation()
                    .getDistance(futureTurretPose.getTranslation());

            poses[i] = new Pose2d(futureTurretPose.getX(), futureTurretPose.getY(), futureTurretPose.getRotation());
        }

        Logger.recordOutput("TurretPose", futureTurretPose);
        Logger.recordOutput("TurretPoses", poses);

        final Pose2d futureRobotPose = futureTurretPose.transformBy(turretOffset.inverse());

        Logger.recordOutput("RobotPose", futureRobotPose);

        return StaticShot.getParameters(
                futureRobotPose,
                futureTurretPose,
                robotRelativeSpeeds,
                targetPose
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
            return MovingTOFShot.getParameters(
                    robotPose,
                    toTurretPoseFn.apply(robotPose),
                    robotRelativeSpeedsSupplier.get(),
                    targetPoseSupplier.get()
            );
        };
    }
}
