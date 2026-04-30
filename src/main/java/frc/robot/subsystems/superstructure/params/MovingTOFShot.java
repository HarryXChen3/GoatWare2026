package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.utils.control.DeltaTime;

public final class MovingTOFShot implements ShotProvider<ShotProvider.Kind.Moving> {
    private static final double LookaheadSeconds = 0.025;

    private final DeltaTime deltaTime = new DeltaTime();
    private final LinearFilter turretOmegaFilter =
            LinearFilter.movingAverage((int)(0.1 / Constants.LOOP_PERIOD_SECONDS));
    private double lastRobotTargetAngleRads = 0;

    @Override
    public ShotParameters getParameters(
            final Pose2d robotPose,
            final Transform2d robotToShooter,
            final ChassisSpeeds robotSpeeds,
            final Pose2d targetPose
    ) {
        final Pose2d lookaheadRobotPose = robotPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * LookaheadSeconds,
                robotSpeeds.vyMetersPerSecond * LookaheadSeconds,
                robotSpeeds.omegaRadiansPerSecond * LookaheadSeconds
        ));
        final Pose2d lookaheadShooterPose = lookaheadRobotPose.transformBy(robotToShooter);
        final double distance = lookaheadShooterPose.getTranslation()
                .getDistance(targetPose.getTranslation());

        final Rotation2d robotAngle = lookaheadRobotPose.getRotation();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotAngle);
        final ChassisSpeeds shooterFieldVelocity =
                MovingUtils.getShooterFieldSpeeds(lookaheadRobotPose, robotToShooter, fieldSpeeds);

        double timeOfFlight;
        Pose2d futureShooterPose = lookaheadShooterPose;
        double futureDistance = distance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = ShotParameters.getTimeOfFlight(futureDistance);

            final Translation2d delta = new Translation2d(
                    shooterFieldVelocity.vxMetersPerSecond * timeOfFlight,
                    shooterFieldVelocity.vyMetersPerSecond * timeOfFlight
            );
            futureShooterPose = new Pose2d(
                    lookaheadShooterPose.getTranslation().plus(delta),
                    lookaheadShooterPose.getRotation()
            );
            futureDistance = futureShooterPose.getTranslation()
                    .getDistance(targetPose.getTranslation());
        }

        final double dt = deltaTime.get();
        final Pose2d futureRobotPose = futureShooterPose.transformBy(robotToShooter.inverse());
        final Rotation2d robotTargetAngle =
                StaticShot.anglePointingShooter(futureRobotPose, robotToShooter, targetPose);
        final double robotTargetAngleRads = robotTargetAngle.getRadians();
        final double robotOmegaRadsPerSec = turretOmegaFilter.calculate(
                MathUtil.angleModulus(robotTargetAngleRads - lastRobotTargetAngleRads)
                        / dt
        );
        lastRobotTargetAngleRads = robotTargetAngleRads;

        return new ShotParameters(
                ShotParameters.getShot(futureDistance),
                robotTargetAngle,
                robotOmegaRadsPerSec
        );
    }
}
