package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public double bottomPositionRots = 0;
        public double bottomVelocityRotsPerSec = 0;
        public double bottomVoltage = 0;
        public double bottomTorqueCurrentAmps = 0;
        public double bottomTempCelsius = 0;

        public double verticalPositionRots = 0;
        public double verticalVelocityRotsPerSec = 0;
        public double verticalVoltage = 0;
        public double verticalTorqueCurrentAmps = 0;
        public double verticalTempCelsius = 0;
    }

    default void updateInputs(final IndexerIOInputs inputs) {}

    default void config() {}

    default void toIndexerVelocity(final double indexerVelocityRotsPerSec) {}

    default void toIndexerVoltage(final double hoodVolts) {}
}
