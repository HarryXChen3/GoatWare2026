package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;

public interface SimConstants {
    // Assume 2mOhm resistance for voltage drop calculation
    double MOTOR_RESISTANCE = 0.002;

    interface CTRE {
        boolean DISABLE_NEUTRAL_MODE_IN_SIM = false;
        double CONFIG_TIMEOUT_SECONDS = 0.2;
    }

    interface Intake {

    }

    interface Hopper {
        Translation3d OCTOPUS_ORIGIN_OFFSET = new Translation3d(0.122, 0, 0);
    }

    interface HopperExtension {

    }

    interface Turret {
        Translation3d ORIGIN_OFFSET = new Translation3d(-0.127, 0, 0.386);
    }

    interface Hood {
        Translation3d TURRET_OFFSET = new Translation3d(0.121, 0, 0.054);
    }
}
