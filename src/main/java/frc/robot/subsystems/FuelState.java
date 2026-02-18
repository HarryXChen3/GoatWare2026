package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
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
    private final CircularBuffer<Fuel> simFuel;
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

        this.simFuel = new CircularBuffer<>(50);
        for (int i = 0; i < 50; i++) {
            simFuel.addFirst(new Fuel(Translation3d.kZero));
        }

        this.hasSimFuel = group.t("hasSimFuel", () -> simFuelCount > 0);
        this.hasFuel = group.t("hasFuel", indexer::isFeederTOFDetected)
                .debounce(0.5, Debouncer.DebounceType.kFalling);

        configureStateTriggers();
        switch (mode) {
            case SIM, REPLAY -> configureSimTriggers();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/HasFuel", hasFuel);
        Logger.recordOutput(LogKey + "/SimFuelCount", simFuelCount);
        Logger.recordOutput(LogKey + "/HasSimFuel", hasSimFuel);

        switch (mode) {
            case SIM, REPLAY -> {
                final int simFuelCount = simFuel.size();
                final Pose3d[] simFuelPoses = new Pose3d[simFuelCount];

                final double dtSeconds = deltaTime.get();
                for (int i = 0; i < simFuel.size(); i++) {
                    final Fuel fuel = simFuel.get(i);
                    fuel.update(dtSeconds);

                    simFuelPoses[i] = fuel.getPose();
                }

                Logger.recordOutput(LogKey + "/SimFuelPoses", simFuelPoses);
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
                                    ));

                            final Fuel fuel = simFuel.removeLast();
                            fuel.at(new Pose3d(swerve.getPose()).rotateBy(new Rotation3d(0, Math.PI / 2, 0)), 5);
                            simFuel.addFirst(fuel);

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

    private static class Fuel {
        private static final Translation3d ForwardAxis = new Translation3d(1, 0, 0);
        private static final double GravityMetersPerSecSquared = 9.81;

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
            x += vx * dtSeconds;
            y += vy * dtSeconds;
            z += vz * dtSeconds
                    - 0.5 * GravityMetersPerSecSquared * dtSeconds * dtSeconds;

            vz -= GravityMetersPerSecSquared * dtSeconds;
        }

        public Pose3d getPose() {
            return new Pose3d(x, y, z, Rotation3d.kZero);
        }
    }
}
