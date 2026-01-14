package frc.robot.constants;

public class HardwareConstants {
    public static final int PowerDistributionHub = 1;

    public enum CANBus {
        RIO("rio"),
        CANIVORE("CANivore");

        public final String name;
        CANBus(final String name) {
            this.name = name;
        }

        public com.ctre.phoenix6.CANBus toPhoenix6CANBus() {
            return new com.ctre.phoenix6.CANBus(name);
        }
    }

    public record TurretConstants(
            CANBus CANBus,
            int motorId,
            int primaryCANcoderId,
            int secondaryCANcoderId,
            double primaryCANcoderOffsetRots,
            double secondaryCANcoderOffsetRots,
            double forwardLimitRots,
            double reverseLimitRots,
            int drivingGearTeeth,
            int drivenTurretGearTeeth,
            int primaryCANcoderGearTeeth,
            int secondaryCANcoderGearTeeth,
            double motorToGearboxGearing,
            double gearboxToTurretGearing,
            double primaryCANcoderGearing,
            double secondaryCANcoderGearing
    ) {
        public TurretConstants(
                CANBus CANBus,
                int motorId,
                int primaryCANcoderId,
                int secondaryCANcoderId,
                double primaryCANcoderOffsetRots,
                double secondaryCANcoderOffsetRots,
                double forwardLimitRots,
                double reverseLimitRots,
                int drivingGearTeeth,
                int drivenTurretGearTeeth,
                int primaryCANcoderGearTeeth,
                int secondaryCANcoderGearTeeth,
                double motorToGearboxGearing
        ) {
            this(
                    CANBus,
                    motorId,
                    primaryCANcoderId,
                    secondaryCANcoderId,
                    primaryCANcoderOffsetRots,
                    secondaryCANcoderOffsetRots,
                    forwardLimitRots,
                    reverseLimitRots,
                    drivingGearTeeth,
                    drivenTurretGearTeeth,
                    primaryCANcoderGearTeeth,
                    secondaryCANcoderGearTeeth,
                    motorToGearboxGearing,
                    ((double) drivenTurretGearTeeth) / drivingGearTeeth,
                    ((double) primaryCANcoderGearTeeth) / drivenTurretGearTeeth,
                    ((double) secondaryCANcoderGearTeeth) / drivenTurretGearTeeth
            );
        }
    }

    public static final TurretConstants TURRET_CONSTANTS = new TurretConstants(
            CANBus.RIO,
            14,
            15,
            16,
            0.0,
            0.0,
            2.0,
            0.0,
            10,
            125,
            31,
            37,
            52.0 / 14.0
    );

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
            (16.0 / 56.0) * (14.0 / 42.0) * (14.0 / 510.0),
            0.1,
            0
    );

    public record ShooterConstants(
            CANBus CANBus,
            int masterId,
            int followerId,
            double gearing
    ) {}

    public static final ShooterConstants SHOOTER_CONSTANTS = new ShooterConstants(
            CANBus.RIO,
            18,
            19,
            2
    );

    public record IntakeConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final IntakeConstants INTAKE_CONSTANTS = new IntakeConstants(
            CANBus.CANIVORE,
            20,
            2
    );

    public record IntakeSlideConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final IntakeSlideConstants INTAKE_SLIDE_CONSTANTS = new IntakeSlideConstants(
            CANBus.CANIVORE,
            21,
            80

    );
}
