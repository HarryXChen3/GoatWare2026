package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.Objects;

public class HardwareConstants {
    public static final int PowerDistributionHub = 1;

    public enum CANBus {
        RIO("rio"),
        CANIVORE("Raptor"),
        TURRET("TURRET");

        private static final HashMap<String, CANBus> P6BusNameToCANBus = new HashMap<>();
        static {
            for (final CANBus bus : CANBus.values()) {
                P6BusNameToCANBus.put(bus.p6Bus.getName(), bus);
            }
        }

        public final String id;
        public final com.ctre.phoenix6.CANBus p6Bus;
        
        CANBus(final String id) {
            this.id = id;
            this.p6Bus = new com.ctre.phoenix6.CANBus(id);
        }

        public static CANBus fromPhoenix6CANBus(final com.ctre.phoenix6.CANBus bus) {
            return Objects.requireNonNull(
                    P6BusNameToCANBus.get(bus.getName()),
                    () -> String.format("Could not get CANBus: %s", bus.getName())
            );
        }
    }

    public record TurretConstants(
            CANBus CANBus,
            int motorId,
            int CANcoderId,
            double motorEncoderOffsetRots,
            double CANcoderOffsetRots,
            double forwardLimitRots,
            double reverseLimitRots,
            int drivenTurretGearTeeth,
            int motorPinionTeeth,
            int CANcoderPinionTeeth,
            double motorToTurretGearing,
            double CANcoderGearing,
            Transform2d offsetFromCenter
    ) {
        public TurretConstants(
                CANBus CANBus,
                int motorId,
                int CANcoderId,
                double motorEncoderOffsetRots,
                double CANcoderOffsetRots,
                double forwardLimitRots,
                double reverseLimitRots,
                int drivenTurretGearTeeth,
                int motorPinionTeeth,
                int CANcoderPinionTeeth,
                Transform2d offsetFromCenter
        ) {
            this(
                    CANBus,
                    motorId,
                    CANcoderId,
                    motorEncoderOffsetRots,
                    CANcoderOffsetRots,
                    forwardLimitRots,
                    reverseLimitRots,
                    drivenTurretGearTeeth,
                    motorPinionTeeth,
                    CANcoderPinionTeeth,
                    ((double) drivenTurretGearTeeth) / motorPinionTeeth,
                    ((double) drivenTurretGearTeeth) / CANcoderPinionTeeth,
                    offsetFromCenter
            );
        }
    }

    public static final TurretConstants TURRET_CONSTANTS = new TurretConstants(
            CANBus.CANIVORE,
            55,
            56,
            -2.53369140625,
            Units.degreesToRotations(-425.830078125),
            Units.degreesToRotations(160.0),
            Units.degreesToRotations(-240.0),
            280,
            15,
            29,
            new Transform2d(-0.12065, -0.1825625, Rotation2d.kZero)
    );

    public record HoodConstants(
            CANBus CANBus,
            int motorId,
            double gearing,
            double upperLimitRots,
            double lowerLimitRots
    ) {}

    public static final HoodConstants HOOD_CONSTANTS = new HoodConstants(
            CANBus.TURRET,
            1,
            26.8,
            0.155,
            0
    );

    public record ShooterConstants(
            CANBus CANBus,
            int masterId,
            int followerId,
            double gearing
    ) {}

    public static final ShooterConstants SHOOTER_CONSTANTS = new ShooterConstants(
            CANBus.TURRET,
            0,
            2,
            1.2
    );

    public record IntakeRollersConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final IntakeRollersConstants INTAKE_CONSTANTS = new IntakeRollersConstants(
            CANBus.CANIVORE,
            51,
            2.472
    );

    public record IntakePivotConstants(
            CANBus CANBus,
            int motorId,
            double gearing,
            double forwardLimitRots,
            double reverseLimitRots
    ) {}

    public static final IntakePivotConstants INTAKE_PIVOT_CONSTANTS = new IntakePivotConstants(
            CANBus.CANIVORE,
            48,
            22.5,
            0.043105,
            -0.314453
    );

    public record FeederConstants(
            CANBus CANBus,
            int motorId,
            double gearing
    ) {}

    public static final FeederConstants FEEDER_CONSTANTS = new FeederConstants(
            CANBus.CANIVORE,
            60,
            2.33
    );

    public record HopperConstants(
            CANBus CANBus,
            int motor1Id,
            int motor2Id,
            double gearing
    ) {}

    public static final HopperConstants HOPPER_CONSTANTS = new HopperConstants(
            CANBus.CANIVORE,
            9,
            32,
            13.54
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
