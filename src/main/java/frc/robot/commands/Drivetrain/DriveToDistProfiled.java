package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveToDistProfiled extends ProfiledPIDCommand {

    public DriveToDistProfiled(Drivetrain drivetrain, Limelight limelight, double targetDist) {
        super(
            new ProfiledPIDController(
                Constants.Drivetrain.DIST_PID[0],
                Constants.Drivetrain.DIST_PID[1],
                Constants.Drivetrain.DIST_PID[2],
                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_DIST_VEL, Constants.Drivetrain.MAX_DIST_ACCEL)
            ),
            limelight::getDistance,
            targetDist,
            (output, setpoint) -> {
                System.out.println(-output);
                // drivetrain.regArcadeDrive(-output, 0.0);
            },
            drivetrain
        );

        getController().setTolerance(
            Constants.Drivetrain.DIST_TOLERANCE
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }

}