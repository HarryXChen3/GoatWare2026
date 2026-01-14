package frc.robot.subsystems.climb.stinger;

import org.littletonrobotics.junction.AutoLog;

public interface StingerIO {
    @AutoLog
    class StingerIOInputs {
        public double motorPositionRots = 0;
        public double motorVelocityRotsPerSec = 0;
        public double motorVoltage = 0;
        public double motorTorqueCurrentAmps = 0;
        public double motorTempCelsius = 0;
    }

    default void updateInputs(final StingerIOInputs inputs) {}

    default void config() {}

    default void toIntakeArmPosition(final double intakeArmPositionRots) {}

    default void toIntakeArmVoltage(final double intakeArmVolts) {}
}
