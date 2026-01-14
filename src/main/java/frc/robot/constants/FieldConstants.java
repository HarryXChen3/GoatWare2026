package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class FieldConstants {
    public static final double FIELD_LENGTH_X_METERS = Units.inchesToMeters(690.876);
    public static final double FIELD_WIDTH_Y_METERS = Units.inchesToMeters(317);
    public static final Pose2d RED_ORIGIN = new Pose2d(FIELD_LENGTH_X_METERS, FIELD_WIDTH_Y_METERS, Rotation2d.k180deg);

    public static final Pose2d BLUE_HUB_POSE = new Pose2d(
            4.626,
            4.033,
            Rotation2d.kZero
    );

    public static final Pose2d RED_HUB_POSE = BLUE_HUB_POSE.relativeTo(RED_ORIGIN);

    private static <T> T getAllianceFlipped(final T blueAlliance, final T redAlliance) {
        return Robot.IsRedAlliance.getAsBoolean() ? redAlliance : blueAlliance;
    }

    public static Pose2d getHubPose() {
        return getAllianceFlipped(BLUE_HUB_POSE, RED_HUB_POSE);
    }

}
