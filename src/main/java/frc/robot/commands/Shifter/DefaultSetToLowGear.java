package frc.robot.commands.Shifter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shifter;

public class DefaultSetToLowGear extends CommandBase {

    private final Shifter m_shifter;

    public DefaultSetToLowGear(Shifter shifter) {
        m_shifter = shifter;
        addRequirements(shifter);
    }

    @Override
    public void execute() {
        m_shifter.setToLowGear();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}