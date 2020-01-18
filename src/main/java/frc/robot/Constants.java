/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.utils.NamedID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class General {
        public static final String DEPLOY_PATH = Filesystem.getDeployDirectory().toString();
        public static final String SIM_PATH = Filesystem.getLaunchDirectory().toString();
    }

    public static final class Drivetrain {
        public static final NamedID LEFT_DRIVE_MASTER_ID = new NamedID("Left-Drive-Master-ID", 10);
        public static final NamedID LEFT_DRIVE_ID = new NamedID("Left-Drive-ID", 13);
        public static final NamedID RIGHT_DRIVE_MASTER_ID = new NamedID("Right-Drive-Master-ID", 12);
        public static final NamedID RIGHT_DRIVE_ID = new NamedID("Right-Drive-ID", 11);
    }
    public static final class Turret {}
    public static final class Intake {}
    public static final class Shooter {
        public static final Double FULL_SEND_SPEED = 1.0;
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
    }
}
