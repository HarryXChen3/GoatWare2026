package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class ClimbIOInputs {
        public double motorPositionRots = 0;
        public double motorVelocityRotsPerSec = 0;
        public double motorVoltage = 0;
        public double motorTorqueCurrentAmps = 0;
        public double motorTempCelsius = 0;
    }

    default void updateInputs(final ClimbIOInputs inputs) {}

    default void config() {}

    default void deployToPosition(final double deployPositionRots) {}

    default void climbToPosition(final double climbPositionRots) {}

    default void toClimbTorqueCurrent(final double climbTorqueCurrentAmps) {}

    default void toClimbVoltage(final double climbVolts) {}
}
