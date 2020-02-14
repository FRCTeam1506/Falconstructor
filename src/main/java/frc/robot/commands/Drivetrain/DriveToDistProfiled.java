package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistProfiled extends ProfiledPIDCommand {

    public DriveToDistProfiled(Drivetrain drivetrain, double targetDistMeters) {
        super(
            new ProfiledPIDController(
                Constants.Drivetrain.DIST_PID[0],
                Constants.Drivetrain.DIST_PID[1],
                Constants.Drivetrain.DIST_PID[2],
                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_DIST_VEL, Constants.Drivetrain.MAX_DIST_ACCEL)
            ),
            drivetrain::getAverageEncoderDistanceMeters,
            targetDistMeters,
            (output, setpoint) -> drivetrain.regArcadeDrive(output, 0.0),
            drivetrain
        );

        getController().setTolerance(
            Constants.Drivetrain.DIST_TOLERANCE,
            Constants.Drivetrain.DIST_RATE_TOLERANCE
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }

}