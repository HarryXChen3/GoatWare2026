package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public double motorPositionRots = 0;
        public double motorRotorPositionRots = 0;
        public double motorVelocityRotsPerSec = 0;
        public double motorVoltage = 0;
        public double motorTorqueCurrentAmps = 0;
        public double motorTempCelsius = 0;

        public double CANcoderPositionRots = 0;

        public boolean motorConnected = false;
        public boolean CANcoderConnected = false;
        public boolean positionSeeded = false;
    }

    default void updateInputs(final TurretIOInputs inputs) {}

    default void config() {}

    default void seedTurretPosition(final Rotation2d turretPosition) {}

    default void trackTurretPosition(final double turretPositionRots, final double turretVelocityRotsPerSec) {}

    default void toTurretPosition(final double turretPositionRots) {}

    default void toTurretVoltage(final double turretVolts) {}
}
