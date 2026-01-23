package frc.robot.subsystems.indexers.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    class FeederIOInputs {
        public double rollerPositionRots = 0;
        public double rollerVelocityRotsPerSec = 0;
        public double rollerVoltage = 0;
        public double rollerTorqueCurrentAmps = 0;
        public double rollerTempCelsius = 0;
    }

    default void updateInputs(final FeederIOInputs inputs) {}

    default void config() {}

    default void toFeederVelocity(final double feederVelocityRotsPerSec) {}

    default void toFeederVoltage(final double feederVolts) {}
}
