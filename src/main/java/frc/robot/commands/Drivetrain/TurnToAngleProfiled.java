package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TurnToAngleProfiled extends ProfiledPIDCommand {

    public TurnToAngleProfiled(Drivetrain drivetrain, Limelight limelight) {
        super(
            new ProfiledPIDController(
                Constants.Drivetrain.HEADING_PID[0], 
                Constants.Drivetrain.HEADING_PID[1], 
                Constants.Drivetrain.HEADING_PID[2],
                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_TURN_RATE, Constants.Drivetrain.MAX_TURN_ACCEL)
            ),
            limelight::getX,
            0.0,
            (output, setpoint) -> {
                System.out.println(-output);
                drivetrain.regArcadeDrive(0, -output);
            },
            limelight
        );

        getController().enableContinuousInput(-27.0, 27.0);
        getController().setTolerance(
            Constants.Drivetrain.TURN_TOLERANCE
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}