package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveToDist extends PIDCommand {

    public DriveToDist(Drivetrain drivetrain, Limelight limelight) {
        super(
            new PIDController(
                Constants.Drivetrain.DIST_PID[0],
                Constants.Drivetrain.DIST_PID[1],
                Constants.Drivetrain.DIST_PID[2]
            ),
            limelight::getDistance,
            5000.0,
            output -> {
                System.out.println(-output);
                drivetrain.regArcadeDrive(-output, 0.0);
            },
            drivetrain
        );

        getController().enableContinuousInput(-15000.0, 15000.0);
        getController().setTolerance(
            Constants.Drivetrain.DIST_TOLERANCE
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}