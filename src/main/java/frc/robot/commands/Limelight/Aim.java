package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Limelight;

public class Aim extends CommandBase {

    private final Turret turret;
    private final Limelight limelight;

    public Aim(Turret m_turret, Limelight m_limelight) {
        turret = m_turret;
        limelight = m_limelight;
        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        if(limelight.getPipeline() != 5) {
            limelight.setPipeline(5);
        }
    }

    @Override
    public void execute() {
        System.out.println("Executing ...");
        if(limelight.isTargetFound()) {
            Double variation = limelight.getX();
            if(!((variation > -Constants.Limelight.THRESHOLD) && (variation < Constants.Limelight.THRESHOLD))) {
                Double rot = variation * Constants.Limelight.CONVERSION;
                System.out.println("Rot: " + rot);
                // turret.goToPos(pos);
            }
        }
    }
}