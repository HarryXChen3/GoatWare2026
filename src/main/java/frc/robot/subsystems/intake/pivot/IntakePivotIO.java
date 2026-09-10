package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
    @AutoLog
    class IntakePivotIOInputs {
        public double positionRots = 0;
        public double velocityRotsPerSec = 0;
        public double voltage = 0;
        public double statorCurrentAmps = 0;
        public double tempCelsius = 0;
        public boolean connected = false;
    }

    default void updateInputs(final IntakePivotIOInputs inputs) {}

    default void config() {}

    default void toPosition(final double positionRots) {}

    default void toProfiledPosition(
            final double positionRots,
            final double maxVelocityRotsPerSec,
            final double maxAccelRotsPerSecSq
    ) {}

    default void toVoltage(final double volts) {}

    default void setPosition(final double positionRots) {}
}
