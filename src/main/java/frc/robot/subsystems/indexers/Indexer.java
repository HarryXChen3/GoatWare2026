package frc.robot.subsystems.indexers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexers.feeder.Feeder;
import frc.robot.subsystems.indexers.hopper.Hopper;

public class Indexer {
    private final Hopper hopper;
    private final Feeder feeder;

    public final Trigger hasFuel = new Trigger(() -> true);

    public Indexer(final Hopper hopper, final Feeder feeder) {
        this.hopper = hopper;
        this.feeder = feeder;
    }

    public Command runFeed() {
        return Commands.run(() -> {});
    }
}
