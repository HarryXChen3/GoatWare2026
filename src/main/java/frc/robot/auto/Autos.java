package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

public class Autos {
    public static final String LogKey = "Auto";

    private final Swerve swerve;
    private final AutoFactory autoFactory;

    public Autos(
            final Swerve swerve,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.autoFactory = new AutoFactory(
                swerve::getPose,
                photonVision::resetPose,
                swerve::followChoreoSample,
                true,
                swerve,
                (trajectory, trajectoryRunning) -> {
                    Logger.recordOutput(
                            LogKey + "/Trajectory/Path",
                            (Robot.IsRedAlliance.getAsBoolean() ? trajectory.flipped() : trajectory).getPoses()
                    );

                    Logger.recordOutput(
                            LogKey + "/Trajectory/Name",
                            trajectory.name()
                    );

                    Logger.recordOutput(
                            LogKey + "/Trajectory/Running",
                            trajectoryRunning
                    );
                }
        );
    }

    private Command runStartingTrajectory(final AutoTrajectory startingTrajectory) {
        return Commands.sequence(
                startingTrajectory.resetOdometry(),
                startingTrajectory.cmd()
        );
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                Commands.waitUntil(RobotModeTriggers.autonomous().negate())
        );

        return routine;
    }

    public AutoRoutine upAndAtEm() {
        final AutoRoutine routine = autoFactory.newRoutine("UpAndAtEm");
        final AutoTrajectory upAndAtEm = routine.trajectory("UpAndAtEm");

        routine.active().onTrue(runStartingTrajectory(upAndAtEm));
        upAndAtEm.done().onTrue(swerve.wheelXCommand());

        return routine;
    }

    public AutoRoutine downAndAtEm() {
        final AutoRoutine routine = autoFactory.newRoutine("DownAndAtEm");
        final AutoTrajectory downAndAtEm = routine.trajectory("DownAndAtEm");

        routine.active().onTrue(runStartingTrajectory(downAndAtEm));
        downAndAtEm.done().onTrue(swerve.wheelXCommand());

        return routine;
    }
}
