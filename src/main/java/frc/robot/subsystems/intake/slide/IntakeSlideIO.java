package frc.robot.subsystems.intake.slide;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSlideIO {
    @AutoLog
    class IntakeSlideIOInputs {
        public double slidePositionRots = 0;
        public double slideVelocityRotsPerSec = 0;
        public double slideVoltage = 0;
        public double slideTorqueCurrentAmps = 0;
        public double slideTempCelsius = 0;
    }

    default void updateInputs(final IntakeSlideIOInputs inputs) {}

    default void config() {}

    default void toSlidePosition(final double slidePositionRots) {}

    default void holdSlidePosition(final double slidePositionRots) {}

    default void toSlideTorqueCurrent(final double slideTorqueCurrentAmps) {}

    default void toSlideVoltage(final double slideVolts) {}
}
