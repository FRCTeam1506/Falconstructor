package frc.robot.commands.Shifter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shifter;

public class DefaultSetToHighGear extends CommandBase {

    private final Shifter m_shifter;

    public DefaultSetToHighGear(Shifter shifter) {
        m_shifter = shifter;
        addRequirements(shifter);
    }

    @Override
    public void execute() {
        m_shifter.setToHighGear();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}