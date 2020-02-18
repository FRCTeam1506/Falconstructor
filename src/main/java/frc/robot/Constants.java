/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.utils.NamedID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static enum AutonTrajectories {
        Baseline,
        Test,
        Ball5,
        Ball10
    }

    public static final class General {
        public static final String DEPLOY_PATH = Filesystem.getDeployDirectory().toString();
        public static final String SIM_PATH = Filesystem.getLaunchDirectory().toString();
    }

    public static final class Limelight {
        public static final Double kP = -0.073; // -0.1
        public static final Double MIN_PWR = 0.05;
        public static final Double THRESHOLD = 0.1;
        public static final Double CONVERSION = 0.03355;
        public static final Double LIMIT = 0.4;
    }

    public static final class Drivetrain {
        public static final NamedID LEFT_DRIVE_MASTER_ID = new NamedID("Left-Drive-Master-ID", 10);
        public static final NamedID LEFT_DRIVE_ID = new NamedID("Left-Drive-ID", 13);
        public static final NamedID RIGHT_DRIVE_MASTER_ID = new NamedID("Right-Drive-Master-ID", 12);
        public static final NamedID RIGHT_DRIVE_ID = new NamedID("Right-Drive-ID", 11);

        public static final Double LEFT_TICKS_PER_REV = 150175.0 / Units.inchesToMeters(127); // 18600.0
        public static final Double RIGHT_TICKS_PER_REV = 162088.0 / Units.inchesToMeters(141); // 9326 19500.0

        public static final Double WHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * 0.0762;
        public static final Double GEAR_RATIO = 1 / 9.46969696; // 9.4696969 // 1 / 0.66

        public static final Double kTrackwidthMeters = 5.760433057155806; // 0.61 5.760433057155806
        public static final Double MAX_VELOCITY = 2.0;
        public static final Double MAX_ACCELERATION = 0.5;

        public static final Double MAX_VOLTS = 100.0;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final Double kS = 0.37; // 0.37
        public static final Double kV = 0.224; // 0.224
        public static final Double kA = 0.0109; // 0.0109
        public static final Double kP = 0.000439; // 0.000439
        public static final Double kD = 0.000183; // 0.000183

        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                kS,
                kV,
                kA
            ),
            kDriveKinematics, 10
        );

        public static final TrajectoryConfig config = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION).setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint);
    
        public static final double[] HEADING_PID = {0.015, 0.03, 0.0031}; // 0.02 0.001

        public static final double TURN_TOLERANCE = 0.01;
        public static final double TURN_RATE_TOLERANCE = 10.0;

        public static final double MAX_TURN_RATE = 100.0;
        public static final double MAX_TURN_ACCEL = 300.0;

        public static final double[] DIST_PID = {0.00009, 0.00003, 0.000001}; 
        // public static final double[] DIST_PID = {0.00013, 0.00003, 0.000001};

        public static final double DIST_TOLERANCE = 50.0;
        public static final double DIST_RATE_TOLERANCE = 300.0;

        public static final double MAX_DIST_VEL = 10.0;
        public static final double MAX_DIST_ACCEL = 30.0;
    }
    public static final class Turret {
        public static final NamedID TURRET_ID = new NamedID("Turret-ID", 17);
    }
    public static final class Indexer {
        public static final NamedID INDEXER_ID = new NamedID("Indexer-ID", 8);
    }
    public static final class Intake {
        public static final NamedID INTAKE_ID = new NamedID("Intake-ID", 6);
    }
    public static final class Shooter {
        public static final NamedID SHOOTER_1_ID = new NamedID("Shooter-1-ID", 14);
        public static final NamedID SHOOTER_2_ID = new NamedID("Shooter-2-ID", 15);
        public static final Double FULL_SEND_SPEED = 1.0;
        public static final Double kP = 0.037; // 0.037
        public static final Double kI = 0.0;
        public static final Double kD = 0.0;
        public static final Double kF = 0.049; // 0.06
    }

    public static final class Playstation {
        
        // Driver Controls
        public static final NamedID USBID = new NamedID("Driver-USB-ID", 0);

        // Axis
        public static final NamedID LeftXAxis = new NamedID("Driver-Left-X-Axis", 0);
        public static final NamedID LeftYAxis = new NamedID("Driver-Left-Y-Axis", 1);
        public static final NamedID RightXAxis = new NamedID("Driver-Right-X-Axis", 2);
        public static final NamedID RightYAxis = new NamedID("Driver-Right-Y-Axis", 5);

        // Trigger
        public static final NamedID LeftTrigger = new NamedID("Driver-Left-Trigger", 3);
        public static final NamedID RightTrigger = new NamedID("Driver-Right-Trigger", 4);

        // Bumper
        public static final NamedID LeftBumper = new NamedID("Driver-Left-Bumper", 5);
        public static final NamedID RightBumper = new NamedID("Driver-Right-Bumper", 6);

        // Buttons
        public static final NamedID SquareButton = new NamedID("Driver-Square-Button", 1);
        public static final NamedID XButton = new NamedID("Driver-X-Button", 2);
        public static final NamedID CircleButton = new NamedID("Driver-Circle-Button", 3);
        public static final NamedID TriangleButton = new NamedID("Driver-Triangle-Button", 4);

        public static final NamedID LeftTriggerButton = new NamedID("Driver-Left-Trigger-Button", 7);
        public static final NamedID RightTriggerButton = new NamedID("Driver-Right-Trigger-Button", 8);

        public static final NamedID LeftButton = new NamedID("Driver-Left-Button", 9);
        public static final NamedID RightButton = new NamedID("Driver-Right-Button", 10);

        public static final NamedID LeftJoystickButton = new NamedID("Driver-Left-Joystick-Button", 11);
        public static final NamedID RightJoystickButton = new NamedID("Driver-Right-Joystick-Button", 12);
        public static final NamedID MiddleButton = new NamedID("Driver-Middle-Joystick-Button", 13);
        public static final NamedID BigButton = new NamedID("Driver-Big-Button", 14);

        // POV Button
        public static final NamedID NorthPOVButton = new NamedID("Driver-North-POV-Button", 0);
        public static final NamedID NorthEastPOVButton = new NamedID("Driver-North-East-POV-Button", 45);
        public static final NamedID EastPOVButton = new NamedID("Driver-East-POV-Button", 90);
        public static final NamedID SouthEastPOVButton = new NamedID("Driver-North-POV-Button", 135);
        public static final NamedID SouthPOVButton = new NamedID("Driver-North-POV-Button", 180);
        public static final NamedID SouthWestPOVButton = new NamedID("Driver-North-POV-Button", 225);
        public static final NamedID WestPOVButton = new NamedID("Driver-North-POV-Button", 270);
        public static final NamedID NorthWestPOVButton = new NamedID("Driver-North-POV-Button", 315);
    }
}
