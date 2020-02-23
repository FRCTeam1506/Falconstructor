package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeOutake extends CommandBase {

    private final Intake m_intake;
    private final Double m_power;

    public IntakeOutake(Intake intake) {
        m_intake = intake;
        m_power = 0.65 ; // 0.9
    }

    public IntakeOutake(Intake intake, Double pwr) {
        m_intake = intake;
        m_power = pwr;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.intakeRev(m_power);
    }

}