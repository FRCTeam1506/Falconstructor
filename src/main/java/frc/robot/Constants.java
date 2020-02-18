/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

    public static final class General {}

    public static final class Drivetrain {}

    public static final class Turret {}

    public static final class Shooter {}

    public static final class Intake {}

    public static final class HorizIndexer {}

    public static final class VertIndexer {}

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
