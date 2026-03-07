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
        TimeOfFlightMap.put(1.31, 1.06);
        TimeOfFlightMap.put(2.17, 1.0);
        TimeOfFlightMap.put(2.72, 1.02);
        TimeOfFlightMap.put(3.51, 1.06);
        TimeOfFlightMap.put(4.63, 1.2);
        TimeOfFlightMap.put(4.9, 1.24);
        TimeOfFlightMap.put(5.73, 1.31);
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
        final double robotOmegaRadsPerSec = fieldRelativeSpeeds.omegaRadiansPerSecond;

        final double tangentVx = robotOmegaRadsPerSec *
                new Translation2d(turretOffset.getY(), turretOffset.getX())
                        .rotateBy(robotAngle)
                        .getX();
        final double tangentVy = robotOmegaRadsPerSec *
                turretOffset
                        .getTranslation()
                        .rotateBy(robotAngle)
                        .getX();

        final ChassisSpeeds turretFieldVelocity = new ChassisSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond + tangentVx,
                fieldRelativeSpeeds.vyMetersPerSecond + tangentVy,
                robotOmegaRadsPerSec
        );
        Logger.recordOutput("Speeds", turretFieldVelocity);

        final Pose2d[] poses = new Pose2d[20];

        double timeOfFlight;
        Pose2d futureTurretPose = turretPose;
        double futureDistance = distance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = TimeOfFlightMap.get(futureDistance);

            final Translation2d delta = new Translation2d(
                    turretFieldVelocity.vxMetersPerSecond * timeOfFlight,
                    turretFieldVelocity.vyMetersPerSecond * timeOfFlight
            );
            futureTurretPose = new Pose2d(
                    turretPose.getTranslation().plus(delta),
                    turretPose.getRotation()
            );
            futureDistance = futureTurretPose.getTranslation()
                    .getDistance(targetPose.getTranslation());

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
