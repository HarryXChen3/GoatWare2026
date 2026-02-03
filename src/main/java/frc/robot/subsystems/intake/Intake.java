package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.slide.IntakeSlide;

@SuppressWarnings("ClassCanBeRecord")
public class Intake {
    private final IntakeSlide slide;
    private final IntakeRollers rollers;

    public Intake(
            final IntakeSlide slide,
            final IntakeRollers rollers
    ) {
        this.slide = slide;
        this.rollers = rollers;
    }

    public Command intake() {
        return Commands.parallel(
                slide.toInstantGoal(IntakeSlide.Goal.INTAKE),
                rollers.toGoal(IntakeRollers.Goal.INTAKE)
        );
    }

    public Command deploy() {
        return slide.toInstantGoal(IntakeSlide.Goal.INTAKE);
    }

    public Command stow() {
        return Commands.parallel(
                slide.toInstantGoal(IntakeSlide.Goal.STOW),
                rollers.toInstantGoal(IntakeRollers.Goal.OFF)
        );
    }
}
