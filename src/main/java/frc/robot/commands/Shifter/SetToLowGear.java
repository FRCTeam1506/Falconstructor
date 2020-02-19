package frc.robot.commands.Shifter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shifter;

public class SetToLowGear extends CommandBase {

    private final Shifter m_shifter;

    public SetToLowGear(Shifter shifter) {
        m_shifter = shifter;
        addRequirements(shifter);
    }

    @Override
    public void initialize() {
        m_shifter.setToLowGear();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}