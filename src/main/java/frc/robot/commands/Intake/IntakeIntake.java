package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeIntake extends CommandBase {

    private final Intake m_intake;
    private final DoubleSupplier m_power;

    public IntakeIntake(Intake intake, DoubleSupplier power) {
        m_intake = intake;
        m_power = power;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.intake(m_power.getAsDouble());
    }


}