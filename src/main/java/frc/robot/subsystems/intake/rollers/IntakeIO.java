package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double rollerPositionRots = 0;
        public double rollerVelocityRotsPerSec = 0;
        public double rollerVoltage = 0;
        public double rollerTorqueCurrentAmps = 0;
        public double rollerTempCelsius = 0;
    }

    default void updateInputs(final IntakeIOInputs inputs) {}

    default void config() {}

    default void toIntakeVelocity(final double intakeVelocityRotsPerSec) {}

    default void toIntakeVoltage(final double intakeVolts) {}
}
