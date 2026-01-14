package frc.robot.subsystems.intake.slide;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSlideIO {
    @AutoLog
    class IntakeArmIOInputs {
        public double pivotPositionRots = 0;
        public double pivotVelocityRotsPerSec = 0;
        public double pivotVoltage = 0;
        public double pivotTorqueCurrentAmps = 0;
        public double pivotTempCelsius = 0;

        public double pivotEncoderPositionRots;
    }

    default void updateInputs(final IntakeArmIOInputs inputs) {}

    default void config() {}

    default void toIntakeArmPosition(final double intakeArmPositionRots) {}

    default void toIntakeArmVoltage(final double intakeArmVolts) {}
}
