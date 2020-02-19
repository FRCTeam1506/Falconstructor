package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeIntake extends CommandBase {

    private final Intake m_intake;
    private final Double m_power;

    public IntakeIntake(Intake intake) {
        m_intake = intake;
        m_power = 0.9;
    }

    public IntakeIntake(Intake intake, Double pwr) {
        m_intake = intake;
        m_power = pwr;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.intakeFwd(m_power);
    }

}