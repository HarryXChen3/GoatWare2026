package frc.robot.subsystems.climb.elevator;

import frc.robot.subsystems.intake.slide.IntakeSlideIO;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double motorPositionRots = 0;
        public double motorVelocityRotsPerSec = 0;
        public double motorVoltage = 0;
        public double motorTorqueCurrentAmps = 0;
        public double motorTempCelsius = 0;
    }

    default void updateInputs(final IntakeSlideIO.IntakeArmIOInputs inputs) {}

    default void config() {}

    default void toIntakeArmPosition(final double intakeArmPositionRots) {}

    default void toIntakeArmVoltage(final double intakeArmVolts) {}
}
