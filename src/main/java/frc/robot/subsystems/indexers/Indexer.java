package frc.robot.subsystems.indexers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.SimConstants;
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

    public Pose3d[] getComponentPoses() {
        return new Pose3d[] {
                new Pose3d(SimConstants.Hopper.OCTOPUS_ORIGIN_OFFSET,
                        new Rotation3d(hopper.getSimulatedComponentPosition()))
        };
    }

    public Command toFeed() {
        return Commands.parallel(
                hopper.toGoal(Hopper.Goal.FEED),
                feeder.toGoal(Feeder.Goal.FEED)
        );
    }
}
