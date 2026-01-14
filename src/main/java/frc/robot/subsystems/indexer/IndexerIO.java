package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public double leftPositionRots = 0;
        public double leftVelocityRotsPerSec = 0;
        public double leftVoltage = 0;
        public double leftTorqueCurrentAmps = 0;
        public double leftTempCelsius = 0;

        public double rightPositionRots = 0;
        public double rightVelocityRotsPerSec = 0;
        public double rightVoltage = 0;
        public double rightTorqueCurrentAmps = 0;
        public double rightTempCelsius = 0;
    }

    default void updateInputs(final IndexerIOInputs inputs) {}

    default void config() {}
}
