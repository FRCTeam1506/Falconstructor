package frc.robot.commands.Intake;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class IntakeIntake extends CommandBase {

    private final Intake m_intake;
    private final Double m_power;

    public IntakeIntake(Intake intake) {
        m_intake = intake;
        m_power = 0.5 ; // 0.65
    }

    public IntakeIntake(Intake intake, Double pwr) {
        m_intake = intake;
        m_power = pwr;
        addRequirements(intake);
    }

    private boolean getReverse() {
        return new POVButton(RobotContainer.driver, 0).get();
    }

    @Override
    public void initialize() {
        if(getReverse()) m_intake.intakeRev(m_power);
        else m_intake.intakeFwd(m_power);
    }

    // @Override
    // public void execute() {
        
    // }

}