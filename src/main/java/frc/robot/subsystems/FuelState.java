package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
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

    private final Intake intake;
    private final Indexer indexer;
    private final Superstructure superstructure;

    public final LoggedTrigger hasFuel;

    private int simFuelCount = 0;
    private final LoggedTrigger hasSimFuel;

    public FuelState(
            final Constants.RobotMode mode,
            final Intake intake,
            final Indexer indexer,
            final Superstructure superstructure
    ) {
        this.intake = intake;
        this.indexer = indexer;
        this.superstructure = superstructure;

        this.hasFuel = group.t("hasFuel", indexer::isFeederTOFDetected)
                .debounce(0.5, Debouncer.DebounceType.kFalling);
        this.hasSimFuel = group.t("hasSimFuel", () -> simFuelCount > 0);

        configureStateTriggers();
        switch (mode) {
            case SIM, REPLAY -> configureSimTriggers();
            default -> {}
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/HasFuel", hasFuel);
        Logger.recordOutput(LogKey + "/SimFuelCount", simFuelCount);
        Logger.recordOutput(LogKey + "/HasSimFuel", hasSimFuel);
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
                .and(hasSimFuel)
                .whileTrue(setInterval(
                        1 / fuelFedPerSecond,
                        () -> simFuelCount = Math.max(simFuelCount - 1, 0)
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
}
