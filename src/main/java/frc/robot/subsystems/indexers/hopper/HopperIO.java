package frc.robot.subsystems.indexers.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
        @AutoLog
        class HopperIOInputs {
            public double hopperPositionRots = 0;
            public double hopperVelocityRotsPerSec = 0;
            public double hopperVoltage = 0;
            public double hopperTorqueCurrentAmps = 0;
            public double hopperTempCelsius = 0;
        }

        default void updateInputs(final HopperIOInputs inputs) {}

        default void config() {}

        default void toHopperTorqueCurrent(final double hopperTorqueCurrentAmps) {}

        default void toHopperVoltage(final double hopperVolts) {}
}
