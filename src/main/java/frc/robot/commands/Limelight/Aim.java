package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Aim extends CommandBase {

    private final Drivetrain drivetrain;
    private final Limelight limelight;

    public Aim(Drivetrain m_drivetrain, Limelight m_limelight) {
        drivetrain = m_drivetrain;
        limelight = m_limelight;
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        if(limelight.getPipeline() != 5) {
            System.out.println("Setting Pipline to 5");
            limelight.setPipeline(5);
            System.out.println("Pipeline has been set to 5");
        }
    }

    @Override
    public void execute() {
        System.out.println("Executing ...");
        System.out.println(limelight.isTargetFound());
        if(limelight.isTargetFound()) {
            System.out.println("Made it ...");
            Double variation = limelight.getX();
            if(!((variation > -Constants.Limelight.VARIATION) && (variation < Constants.Limelight.VARIATION))) {
                Double rot = variation * Constants.Limelight.CONVERSION;
                System.out.println("Rot: " + rot);
                drivetrain.arcadeDrive(0.0, -rot * Constants.Limelight.TUNE);
            }
        }
    }
}