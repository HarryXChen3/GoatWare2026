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

    public static ChassisSpeeds getTurretFieldSpeeds(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        return getTurretFieldSpeeds(
                robotPose,
                new Transform2d(turretTranslation.minus(robotPose.getTranslation()), Rotation2d.kZero),
                fieldRelativeSpeeds
        );
    }

    public static ChassisSpeeds getTurretFieldSpeeds(
            final Pose2d robotPose,
            final Transform2d turretOffset,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        final Rotation2d robotAngle = robotPose.getRotation();
        final double robotOmegaRadsPerSec = fieldRelativeSpeeds.omegaRadiansPerSecond;

        final double offsetX = turretOffset.getX();
        final double offsetY = turretOffset.getY();
        final double cos = robotAngle.getCos();
        final double sin = robotAngle.getSin();

        final double tangentVx = robotOmegaRadsPerSec * (offsetY * cos - offsetX * sin);
        final double tangentVy = robotOmegaRadsPerSec * (offsetX * cos - offsetY * sin);

        return new ChassisSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond + tangentVx,
                fieldRelativeSpeeds.vyMetersPerSecond + tangentVy,
                robotOmegaRadsPerSec
        );
    }

    public static ShotParameters getParameters(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final ChassisSpeeds robotRelativeSpeeds,
            final Pose2d targetPose
    ) {
        final Pose2d offsetTurretPose = new Pose2d(turretTranslation, Rotation2d.kZero);
        final double distance = turretTranslation
                .getDistance(targetPose.getTranslation());
        final Rotation2d robotAngle = robotPose.getRotation();
        final ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                robotAngle
        );

        final Transform2d turretOffset = offsetTurretPose.minus(robotPose);
        final ChassisSpeeds turretFieldVelocity = getTurretFieldSpeeds(robotPose, turretOffset, fieldRelativeSpeeds);

        double timeOfFlight;
        Pose2d futureTurretPose = offsetTurretPose;
        double futureDistance = distance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = TimeOfFlightMap.get(futureDistance);

            final Translation2d delta = new Translation2d(
                    turretFieldVelocity.vxMetersPerSecond * timeOfFlight,
                    turretFieldVelocity.vyMetersPerSecond * timeOfFlight
            );
            futureTurretPose = new Pose2d(
                    offsetTurretPose.getTranslation().plus(delta),
                    offsetTurretPose.getRotation()
            );
            futureDistance = futureTurretPose.getTranslation()
                    .getDistance(targetPose.getTranslation());
        }

        final Pose2d futureRobotPose = futureTurretPose.transformBy(turretOffset.inverse());
        return StaticShot.getParameters(
                futureRobotPose,
                futureTurretPose.getTranslation(),
                robotRelativeSpeeds,
                targetPose
        );
    }

    public static Supplier<ShotParameters> parametersSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Function<Pose2d, Translation2d> toTurretFn,
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> {
            final Pose2d robotPose = robotPoseSupplier.get();
            return MovingTOFShot.getParameters(
                    robotPose,
                    toTurretFn.apply(robotPose),
                    robotRelativeSpeedsSupplier.get(),
                    targetPoseSupplier.get()
            );
        };
    }
}
