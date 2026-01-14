package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.superstructure.StaticShot;
import frc.robot.subsystems.superstructure.Superstructure;

public class ShootCommands {
    private final Swerve swerve;
    private final Indexer indexer;
    private final Superstructure superstructure;

    public ShootCommands(
            final Swerve swerve,
            final Indexer indexer,
            final Superstructure superstructure
    ) {
        this.swerve = swerve;
        this.indexer = indexer;
        this.superstructure = superstructure;
    }

    public Command stopAndShoot() {
        return Commands.deadline(
                Commands.sequence(

                ),
                superstructure.runParameters(
                        StaticShot.parametersSupplier(swerve::getPose, () -> Pose2d.kZero)
                ).asProxy(),
                swerve.runWheelXCommand()
        );
    }
}
