package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDist extends PIDCommand {

    public DriveToDist(Drivetrain drivetrain, double targetDistMeters) {
        super(
            new PIDController(
                Constants.Drivetrain.DIST_PID[0],
                Constants.Drivetrain.DIST_PID[1],
                Constants.Drivetrain.DIST_PID[2]
            ),
            drivetrain::getAverageEncoderDistanceMeters,
            targetDistMeters,
            output -> drivetrain.regArcadeDrive(output, 0.0),
            drivetrain
        );

        getController().setTolerance(
            Constants.Drivetrain.DIST_TOLERANCE,
            Constants.Drivetrain.DIST_RATE_TOLERANCE
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}