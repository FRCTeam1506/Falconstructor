package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Extend extends CommandBase {

    private final Intake m_intake;

    public Extend(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.extend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}