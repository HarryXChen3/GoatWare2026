package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import java.util.function.Function;
import java.util.function.Supplier;

public class MovingTOFShot {
    private static final double LookaheadSeconds = 0.025;
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

    public static ChassisSpeeds getShooterFieldSpeeds(
            final Pose2d robotPose,
            final Pose2d shooterPose,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        return getShooterFieldSpeeds(
                robotPose,
                new Transform2d(robotPose, shooterPose),
                fieldRelativeSpeeds
        );
    }

    public static ChassisSpeeds getShooterFieldSpeeds(
            final Pose2d robotPose,
            final Transform2d shooterOffset,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        final double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        final Translation2d rField = shooterOffset
                .getTranslation()
                .rotateBy(robotPose.getRotation());

        final double tangentVx = -omega * rField.getY();
        final double tangentVy = omega * rField.getX();

        return new ChassisSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond + tangentVx,
                fieldRelativeSpeeds.vyMetersPerSecond + tangentVy,
                omega
        );
    }

    public static ShotParameters getParameters(
            final Pose2d robotPose,
            final Pose2d shooterPose,
            final Transform2d shooterOffset,
            final ChassisSpeeds robotSpeeds,
            final Pose2d targetPose
    ) {
        final double distance = shooterPose.getTranslation()
                .getDistance(targetPose.getTranslation());
        final Pose2d lookaheadRobotPose = robotPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * LookaheadSeconds,
                robotSpeeds.vyMetersPerSecond * LookaheadSeconds,
                robotSpeeds.omegaRadiansPerSecond * LookaheadSeconds
        ));

        final Rotation2d robotAngle = lookaheadRobotPose.getRotation();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotSpeeds,
                robotAngle
        );
        final ChassisSpeeds shooterFieldSpeeds = getShooterFieldSpeeds(
                lookaheadRobotPose,
                shooterOffset,
                fieldSpeeds
        );

        double timeOfFlight;
        Pose2d futureShooterPose = shooterPose;
        double futureDistance = distance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = TimeOfFlightMap.get(futureDistance);

            final Translation2d delta = new Translation2d(
                    shooterFieldSpeeds.vxMetersPerSecond * timeOfFlight,
                    shooterFieldSpeeds.vyMetersPerSecond * timeOfFlight
            );
            futureShooterPose = new Pose2d(
                    shooterPose.getTranslation().plus(delta),
                    shooterPose.getRotation()
            );
            futureDistance = futureShooterPose.getTranslation()
                    .getDistance(targetPose.getTranslation());
        }

        final Pose2d futureRobotPose = futureShooterPose.transformBy(shooterOffset.inverse());
        return StaticShot.getParameters(
                futureRobotPose,
                futureShooterPose,
                shooterOffset,
//                robotSpeeds,
                targetPose
        );
    }

    public static Supplier<ShotParameters> parametersSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Function<Pose2d, Pose2d> toShooterFn,
            final Supplier<Transform2d> shooterOffsetSupplier,
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> {
            final Pose2d robotPose = robotPoseSupplier.get();
            return MovingTOFShot.getParameters(
                    robotPose,
                    toShooterFn.apply(robotPose),
                    shooterOffsetSupplier.get(),
                    robotRelativeSpeedsSupplier.get(),
                    targetPoseSupplier.get()
            );
        };
    }
}
