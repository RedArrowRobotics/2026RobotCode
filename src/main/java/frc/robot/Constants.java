package frc.robot;

public final class Constants {
    /** If set to true, enables SysId and more verbose logging. */
    public static final boolean DEBUG_ENABLED = true;
    
    public final class OperatorConstants {
        public static final int DRIVE_JOYSTICK_PORT = 0;
        public static final int COMPONENTS_BOARD_PORT = 1;
    }

    public final class DeviceConstants {
        public static final int FUEL_SHOOTER_MOTOR_ID = 15;
    }

    public final class DriveConstants {
        public static final String SWERVE_CONFIG = "swerve-practice";
    }
    
    public final class FuelShooterConstants {
        public static final double THROWER_POWER = 0.5;
    }
    public final class AgitatorConstants {
        public static final int BELT_MOTOR_1_ID = 20;
        public static final int BELT_MOTOR_2_ID = 21;
        public static final int KICKER_MOTOR_ID = 22;
        public static final double BELT_SPEED = 0.5;
        public static final double KICK_SPEED = 0.5;
    }
}
