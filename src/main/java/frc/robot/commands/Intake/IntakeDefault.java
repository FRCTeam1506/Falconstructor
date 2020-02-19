package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends CommandBase {

    private final Intake m_intake;

    public IntakeDefault(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.retract();
    }

    @Override
    public void execute() {
        m_intake.stopIntake();
    }

}