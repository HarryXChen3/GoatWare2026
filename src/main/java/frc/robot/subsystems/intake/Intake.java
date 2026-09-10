package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.utils.commands.trigger.LoggedTrigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake {
    protected static final String LogKey = "Intake";
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final IntakePivot pivot;
    private final IntakeRollers rollers;

    private boolean intaking = false;
    public final LoggedTrigger isIntaking = group.t("IsIntaking", () -> intaking);

    public Intake(
            final IntakePivot pivot,
            final IntakeRollers rollers
    ) {
        this.pivot = pivot;
        this.rollers = rollers;
    }

    private Command intaking() {
        return startEnd(() -> intaking = true, () -> intaking = false).withName("IntakingImpl");
    }

    public Command intake() {
        return parallel(
                intaking(),
                pivot.toInstantGoal(IntakePivot.Goal.INTAKE),
                sequence(
                        waitUntil(pivot.atGoal(IntakePivot.Goal.INTAKE))
                                .withTimeout(4),
                        rollers.toGoal(IntakeRollers.Goal.INTAKE)
                )
        ).withName("Intaking");
    }

    public Command deploy() {
        return pivot.toInstantGoal(IntakePivot.Goal.INTAKE)
                .withName("DeployIntake");
    }

    public Command stow() {
        return parallel(
                pivot.toInstantGoal(IntakePivot.Goal.STOW),
                rollers.toInstantGoal(IntakeRollers.Goal.OFF)
        ).withName("StowIntake");
    }

    public Command stowFeed() {
        return parallel(
                sequence(
                        pivot.toGoal(IntakePivot.Goal.INTAKE)
                                .withTimeout(1),
                        repeatingSequence(
                                pivot.runGoal(IntakePivot.Goal.GYRATE_MAX)
                                        .until(pivot.atSetpoint)
                                        .withTimeout(0.5),
                                pivot.runGoal(IntakePivot.Goal.GYRATE_MIN)
                                        .until(pivot.atSetpoint)
                                        .withTimeout(0.5)
                        )
                ),
                rollers.toGoal(IntakeRollers.Goal.INTAKE)
        ).withName("StowFeedIntake");
    }
}
