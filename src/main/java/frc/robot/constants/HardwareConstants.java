package frc.robot.constants;

import edu.wpi.first.math.geometry.*;

import java.util.HashMap;
import java.util.Objects;

public class HardwareConstants {
    public static final int PowerDistributionHub = 1;

    public enum CANBus {
        RIO("rio"),
        CANIVORE("CANivore");

        private static final HashMap<String, CANBus> BusNameToCANBus = new HashMap<>();
        static {
            for (final CANBus bus : CANBus.values()) {
                BusNameToCANBus.put(bus.name, bus);
            }
        }

        public final String name;
        CANBus(final String name) {
            this.name = name;
        }

        public static CANBus fromPhoenix6CANBus(final com.ctre.phoenix6.CANBus bus) {
            return Objects.requireNonNull(
                    BusNameToCANBus.get(bus.getName()), () -> String.format("Could not get CANBus: %s", bus.getName()));
        }

        public com.ctre.phoenix6.CANBus toPhoenix6CANBus() {
            return new com.ctre.phoenix6.CANBus(name);
        }
    }

    public record HoodConstants(
            CANBus CANBus,
            int motorId,
            double gearing,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static final HoodConstants HOOD_CONSTANTS = new HoodConstants(
            CANBus.RIO,
            17,
            (40.0 / 12.0) * (15.0 / 20.0) * (180.0 / 10.0),
            0.1,
            0
    );

    public record ShooterConstants(
            CANBus CANBus,
            int masterId,
            int followerId,
            double gearing,
            Transform2d offsetFromCenter
    ) {}

    public static final ShooterConstants SHOOTER_CONSTANTS = new ShooterConstants(
            CANBus.RIO,
            18,
            19,
            2,
            new Transform2d(-0.2305, 0, Rotation2d.kPi)
    );

    public record IntakeRollersConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final IntakeRollersConstants INTAKE_CONSTANTS = new IntakeRollersConstants(
            CANBus.RIO,
            20,
            20.0 / 12.0
    );

    public record IntakeSlideConstants(
            CANBus CANBus,
            int masterMotorId,
            int followerMotorId,
            double averageAxisGearing,
            double differentialAxisGearing,
            double forwardLimitRots,
            double reverseLimitRots
    ) {}

    public static final IntakeSlideConstants INTAKE_SLIDE_CONSTANTS = new IntakeSlideConstants(
            CANBus.CANIVORE,
            21,
            22,
            (60.0 / 12.0) * (40.0 / 18.0),
            1,
            4.4,
            0
    );

    public record FeederConstants(
            CANBus CANBus,
            int motorId,
            int tofId,
            double gearing
    ) {}

    public static final FeederConstants FEEDER_CONSTANTS = new FeederConstants(
            CANBus.CANIVORE,
            23,
            24,
            (36.0 / 12.0) * (24.0 / 18.0)
    );

    public record HopperConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final HopperConstants HOPPER_CONSTANTS = new HopperConstants(
            CANBus.CANIVORE,
            25,
            3
    );

    public record ClimbConstants(
            CANBus CANBus,
            int motorId,
            double gearing,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static final ClimbConstants CLIMB_CONSTANTS = new ClimbConstants(
            CANBus.CANIVORE,
            26,
            48,
            5.05,
            0
    );
}
