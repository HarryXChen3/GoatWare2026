package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

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
            double secondaryCANcoderGearing,
            Transform2d offsetFromCenter
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
                double motorToGearboxGearing,
                Transform2d offsetFromCenter
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
                    ((double) secondaryCANcoderGearTeeth) / drivenTurretGearTeeth,
                    offsetFromCenter
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
            1.0,
            -1.0,
            10,
            80,
            13,
            17,
            36.0 / 12.0,
            new Transform2d(-0.127, 0, Rotation2d.kZero)
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
            (40.0 / 12.0) * (15.0 / 20.0) * (180.0 / 10.0),
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

    public record IntakeRollersConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final IntakeRollersConstants INTAKE_CONSTANTS = new IntakeRollersConstants(
            CANBus.CANIVORE,
            20,
            20.0 / 12.0
    );

    public record IntakeSlideConstants(
            CANBus CANBus,
            int motorId,
            double gearing,
            double forwardLimitRots,
            double reverseLimitRots
    ) {}

    public static final IntakeSlideConstants INTAKE_SLIDE_CONSTANTS = new IntakeSlideConstants(
            CANBus.CANIVORE,
            21,
            (60.0 / 12.0) * (40.0 / 18.0),
            3.9,
            0
    );

    public record FeederConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final FeederConstants FEEDER_CONSTANTS = new FeederConstants(
            CANBus.CANIVORE,
            22,
            (36.0 / 12.0) * (24.0 / 18.0)
    );

    public record HopperConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final HopperConstants HOPPER_CONSTANTS = new HopperConstants(
            CANBus.CANIVORE,
            23,
            3
    );
}
