package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double MotorResistanceOhms = 0.002;

    interface CTRE {
        double ConfigTimeoutSeconds = 0.2;
    }

    interface Hood {
        Translation3d OriginOffset = new Translation3d(-0.2305, 0, 0.4841);
        Transform3d FuelExitOffset = new Transform3d(
                Units.inchesToMeters(4.604),
                0,
                Units.inchesToMeters(-2.125),
                new Rotation3d(0, (-Math.PI / 2) - Units.degreesToRadians(10), 0)
        );

        double WidthMeters = Units.inchesToMeters(23.5);
        double FuelRadiusMeters = Units.inchesToMeters(5.91) / 2;
        double FuelExitLeftYBoundMeters = (WidthMeters / 2) - FuelRadiusMeters;
        double FuelExitRightYBoundMeters = -FuelExitLeftYBoundMeters;
    }

    interface Shooter {
        double WheelRadiusMeters = Units.inchesToMeters(2);
        double WheelCircumferenceMeters = 2 * Math.PI * WheelRadiusMeters;
    }

    interface IntakeSlide {
        double DrivingGearDiameterMeters = Units.inchesToMeters(1);
        double SlideRotationsToLinearDistanceMetersRatio = 2 * Math.PI * (DrivingGearDiameterMeters / 2);

        Pose3d ExtendedPose = new Pose3d(
                Units.inchesToMeters(13.125),
                0,
                0,
                Rotation3d.kZero
        );
        Pose3d RetractedPose = Pose3d.kZero;
    }

    interface HopperExtension {
        Pose3d ExtendedPose = new Pose3d(
                Units.inchesToMeters(11),
                0,
                0,
                Rotation3d.kZero
        );
        Pose3d RetractedPose = Pose3d.kZero;
    }

    interface Climb {
        double ClimbWeightKgs = Units.lbsToKilograms(125);
        double PulleyRadiusMeters = Units.inchesToMeters(0.5);
        double PulleyCircumferenceMeters = 2 * Math.PI * PulleyRadiusMeters;

        Pose3d ExtendedPose = Pose3d.kZero;
        Pose3d RetractedPose = new Pose3d(
                Units.inchesToMeters(6.272),
                0,
                Units.inchesToMeters(-13.9),
                Rotation3d.kZero
        );

        double Stage0MaxExtensionMeters = Units.inchesToMeters(7.652);
    }
}
