package frc.robot.utils.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class FastRepeatCommand extends Command {
    private final Command m_command;
    private boolean m_ended;

    /**
     * Creates a new RepeatCommand. Will run another command repeatedly, restarting it whenever it
     * ends, until this command is interrupted.
     *
     * @param command the command to run repeatedly
     */
    @SuppressWarnings("this-escape")
    public FastRepeatCommand(final Command command) {
        m_command = requireNonNullParam(command, "command", "RepeatCommand");
        CommandScheduler.getInstance().registerComposedCommands(command);
        addRequirements(command.getRequirements());
        setName("Repeat(" + command.getName() + ")");
    }

    @Override
    public void initialize() {
        m_ended = false;
        m_command.initialize();
    }

    @Override
    public void execute() {
        if (m_ended) {
            m_ended = false;
            m_command.initialize();
        }
        m_command.execute();
        if (m_command.isFinished()) {
            m_command.end(false);

            m_command.initialize();
            m_command.execute();

//            m_ended = true;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(final boolean interrupted) {
        // Make sure we didn't already call end() (which would happen if the command finished in the
        // last call to our execute())
        if (!m_ended) {
            m_command.end(interrupted);
            m_ended = true;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_command.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_command.getInterruptionBehavior();
    }

    @Override
    public void initSendable(final SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("command", m_command::getName, null);
    }
}
