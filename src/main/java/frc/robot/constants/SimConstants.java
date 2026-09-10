package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double MotorResistanceOhms = 0.002;

    interface CTRE {
        double ConfigTimeoutSeconds = 0.2;
    }

    interface Hood {
        Translation3d TurretOffset = new Translation3d(0.121, 0, 0.054);
        Transform3d FuelExitOffset = new Transform3d(
                Units.inchesToMeters(-4.604),
                0,
                Units.inchesToMeters(-2.125),
                new Rotation3d(0, (-Math.PI / 2) + Units.degreesToRadians(10), 0)
        );
    }

    interface Turret {
        Translation3d OriginOffset = new Translation3d(-0.12065, -0.1825625, 0.386);
    }

    interface Shooter {
        double WheelRadiusMeters = Units.inchesToMeters(2);
        double WheelCircumferenceMeters = 2 * Math.PI * WheelRadiusMeters;
    }

    interface IntakePivot {
        Translation3d OriginOffset = new Translation3d(0.24375, 0, 0.254);
        Rotation2d ZeroedPositionToSimZero = Rotation2d.fromDegrees(113.2);
    }

//    interface HopperExtension {
//        Pose3d ExtendedPose = new Pose3d(
//                Units.inchesToMeters(12.606),
//                0,
//                0,
//                Rotation3d.kZero
//        );
//        Pose3d RetractedPose = Pose3d.kZero;
//    }

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

    interface Hopper {
        Translation3d OriginOffset = new Translation3d(-0.0273, 0.021, 0.11);
    }
}
