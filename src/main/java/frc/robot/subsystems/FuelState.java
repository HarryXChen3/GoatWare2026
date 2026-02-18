package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.Container;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.ThreadLocalRandom;

public class FuelState extends VirtualSubsystem {
    protected static final String LogKey = "FuelState";
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final DeltaTime deltaTime;
    private final Constants.RobotMode mode;

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final Superstructure superstructure;

    private int simFuelCount = 0;
    private final FuelCache fuelCache;
    private final LoggedTrigger hasSimFuel;

    public final LoggedTrigger hasFuel;

    public FuelState(
            final Constants.RobotMode mode,
            final Swerve swerve,
            final Intake intake,
            final Indexer indexer,
            final Superstructure superstructure
    ) {
        this.deltaTime = new DeltaTime(true);
        this.mode = mode;

        this.swerve = swerve;
        this.intake = intake;
        this.indexer = indexer;
        this.superstructure = superstructure;

        this.hasSimFuel = group.t("hasSimFuel", () -> simFuelCount > 0);
        this.hasFuel = group.t("hasFuel", indexer::isFeederTOFDetected)
                .debounce(0.5, Debouncer.DebounceType.kFalling);

        configureStateTriggers();
        switch (mode) {
            case SIM, REPLAY -> {
                this.fuelCache = new FuelCache(50);
                configureSimTriggers();
            }
            default -> this.fuelCache = null;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/HasFuel", hasFuel);
        Logger.recordOutput(LogKey + "/SimFuelCount", simFuelCount);
        Logger.recordOutput(LogKey + "/HasSimFuel", hasSimFuel);

        switch (mode) {
            case SIM, REPLAY -> {
                fuelCache.periodic(deltaTime.get());
                Logger.recordOutput(LogKey + "/SimFuelPoses", fuelCache.getPoses());
            }
        }
    }

    private void configureStateTriggers() {
        intake.isIntaking.and(hasFuel.negate())
                .whileTrue(indexer.toFeed());
    }

    private void configureSimTriggers() {
        final ThreadLocalRandom random = ThreadLocalRandom.current();

        final double fuelIntakePerSecond = 5;
        intake.isIntaking.whileTrue(setInterval(1 / fuelIntakePerSecond, () -> simFuelCount++));

        final double fuelFedPerSecond = 8;
        indexer.isFeeding
                .and(hasFuel)
                .and(hasSimFuel)
                .whileTrue(setInterval(
                        1 / fuelFedPerSecond,
                        () -> {
                            final Pose3d hoodComponentPose = superstructure.getComponentPoses()[1];
                            final Pose3d hoodPose = new Pose3d(swerve.getPose())
                                    .plus(new Transform3d(
                                            hoodComponentPose.getTranslation(),
                                            hoodComponentPose.getRotation()
                                    ))
                                    .plus(SimConstants.Hood.FuelExitOffset);

                            fuelCache.spawn(hoodPose, 7.5);
                            simFuelCount = Math.max(simFuelCount - 1, 0);
                        }
                ));

        intake.isIntaking
                .and(indexer.isFeeding)
                .and(hasFuel.negate())
                .and(hasSimFuel)
                .onTrue(Commands.sequence(
                        waitRand(random, 0.25, 0.5),
                        Commands.runOnce(() -> indexer.setFeederTOFDetected(true))
                ));
        indexer.isFeeding
                .and(hasSimFuel.negate())
                .onTrue(Commands.runOnce(() -> indexer.setFeederTOFDetected(false)));
    }

    @SuppressWarnings("SameParameterValue")
    private static Command waitRand(
            final ThreadLocalRandom random,
            final double lowerInclusiveSeconds,
            final double upperExclusiveSeconds
    ) {
        return Commands.waitSeconds(random.nextDouble(lowerInclusiveSeconds, upperExclusiveSeconds));
    }

    private static Command setInterval(final double intervalSeconds, final Runnable callback) {
        final Container<Double> tContainer = Container.of(0d);
        final DeltaTime deltaTime = new DeltaTime();

        return Commands.runEnd(
                () -> {
                    double t = tContainer.get();
                    t += deltaTime.get();

                    while (t >= intervalSeconds) {
                        t -= intervalSeconds;
                        callback.run();
                    }
                    tContainer.set(t);
                },
                deltaTime::reset
        );
    }

    private static class FuelCache {
        private static final Translation3d FAR_AWAY =
                new Translation3d(-100, -100, -100);

        private final Fuel[] fuel;
        private int index = 0;

        public FuelCache(final int capacity) {
            fuel = new Fuel[capacity];

            for (int i = 0; i < capacity; i++) {
                fuel[i] = new Fuel(FAR_AWAY);
            }
        }

        public void spawn(final Pose3d pose, final double velocityMetersPerSec) {
            final Fuel cached = fuel[index];
            cached.at(pose, velocityMetersPerSec);
            index = (index + 1) % fuel.length;
        }

        public void periodic(final double dtSeconds) {
            for (final Fuel cached : fuel) {
                if (!cached.active) {
                    continue;
                }

                cached.update(dtSeconds);
                if (cached.getZ() < 0) {
                    cached.discard(FAR_AWAY);
                }
            }
        }

        public Pose3d[] getPoses() {
            final int fuelCount = fuel.length;
            final Pose3d[] poses = new Pose3d[fuelCount];
            for (int i = 0; i < fuel.length; i++) {
                final Fuel cached = fuel[i];
                poses[i] = cached.getPose();
            }

            return poses;
        }

        private static class Fuel {
            private static final Translation3d ForwardAxis = new Translation3d(1, 0, 0);
            private static final double GravityMetersPerSecSquared = 9.81;

            private boolean active;

            private double x;
            private double y;
            private double z;

            private double vx;
            private double vy;
            private double vz;

            public Fuel(final Translation3d pos) {
                this.x = pos.getX();
                this.y = pos.getY();
                this.z = pos.getZ();

                this.vx = 0;
                this.vy = 0;
                this.vz = 0;
            }

            public void at(final Pose3d pose, final double velocityMetersPerSec) {
                active = true;

                x = pose.getX();
                y = pose.getY();
                z = pose.getZ();

                final Translation3d velocity = ForwardAxis.rotateBy(pose.getRotation())
                        .times(velocityMetersPerSec);
                vx = velocity.getX();
                vy = velocity.getY();
                vz = velocity.getZ();
            }

            public void update(final double dtSeconds) {
                if (!active) {
                    return;
                }

                x += vx * dtSeconds;
                y += vy * dtSeconds;
                z += vz * dtSeconds
                        - 0.5 * GravityMetersPerSecSquared * dtSeconds * dtSeconds;

                vz -= GravityMetersPerSecSquared * dtSeconds;
            }

            public void discard(final Translation3d to) {
                active = false;

                x = to.getX();
                y = to.getY();
                z = to.getZ();

                vx = 0.0;
                vy = 0.0;
                vz = 0.0;
            }

            public boolean isActive() {
                return active;
            }

            public double getX() {
                return x;
            }

            public double getY() {
                return y;
            }

            public double getZ() {
                return z;
            }

            public Pose3d getPose() {
                return new Pose3d(x, y, z, Rotation3d.kZero);
            }
        }
    }
}
