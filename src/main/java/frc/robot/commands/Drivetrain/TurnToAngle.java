package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TurnToAngle extends PIDCommand {

    // public TurnToAngle(Drivetrain drivetrain, double targetAngleDegrees) {
    //     super(
    //         new PIDController(
    //             Constants.Drivetrain.HEADING_PID[0], 
    //             Constants.Drivetrain.HEADING_PID[1], 
    //             Constants.Drivetrain.HEADING_PID[2]
    //         ),
    //         drivetrain::getHeading,
    //         targetAngleDegrees,
    //         output -> {
    //             System.out.println(output);
    //             // drivetrain.regArcadeDrive(0, output);
    //         },
    //         drivetrain
    //     );

    //     getController().enableContinuousInput(-180.0, 180.0);
    //     getController().setTolerance(
    //         Constants.Drivetrain.TURN_TOLERANCE
    //     );

    public TurnToAngle(Drivetrain drivetrain, Limelight limelight) {
        super(
            new PIDController(
                Constants.Drivetrain.HEADING_PID[0], 
                Constants.Drivetrain.HEADING_PID[1], 
                Constants.Drivetrain.HEADING_PID[2]
            ),
            limelight::getX,
            0.0,
            output -> {
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
        return getController().atSetpoint();
    }

}