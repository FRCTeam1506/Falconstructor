package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Retract extends CommandBase {

    private final Intake m_intake;

    public Retract(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}