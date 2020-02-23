// //! BROKEN

// package frc.robot.commands.Drivetrain;

// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.Drivetrain;

// public class DriveEncoderDist extends PIDCommand {

//     private double targetAngle;
//     private double speed;
//     private double rotateToAngleRate;
//     private final Drivetrain drivetrain;

//     public DriveEncoderDist(Drivetrain drivetrain, double target) {
//         super(
//             new PIDController(
//                 0.0,0.0,0.0
//             ),
//             targetAngle,
//             rotateToAngleRate,
//             output -> {

//             }

//         );
//     }

//     @Override
//     public void initialize() {
//         targetAngle = drivetrain.getHeading();
//     }

//     @Override
//     public void execute() {
//         drivetrain.tankDrive(speed + rotateToAngleRate, speed - rotateToAngleRate);
//     }

// }