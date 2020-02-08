package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class IntakeDouble extends CommandBase {
    // Declare Variables
    private final Intake m_intake;
    private final Double m_power;

    public IntakeDouble(Intake subsystem, Double power) {
        m_intake = subsystem;
        m_power = power;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        new WaitCommand(2.0);
        m_intake.intake(m_power);
    }
}