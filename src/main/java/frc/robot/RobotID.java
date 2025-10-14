// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotID {
    public static final class DriveBase {

        public static final int PIGEON_ID = 13;

        public static final class FrontLeft {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
        }

        public static final class FrontRight {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
        }

        public static final class BackLeft {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
        }

        public static final class BackRight {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
        }
    }

    public static final class Elevator {
        public static final int ELEVATOR_LEFT_MOTOR = 14;
        public static final int ELEVATOR_RIGHT_MOTOR = 15;
    }

    public static final class Pivot {
        public static final int MOTOR_ID = 32;
    }

    public static final class Climb {
        public static final int MOTOR_ID = 3;
    }

    public static final class Intake {
        public static final int ROLLER_LEFT = 40;
        public static final int ROLLER_RIGHT = 35;
    }
}