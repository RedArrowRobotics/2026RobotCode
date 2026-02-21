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
        public static final int CLIMBER_MOTOR_ID = 19;
    }

    public final class DriveConstants {
        public static final String SWERVE_CONFIG = "swerve-practice";
    }
    
    public final class FuelShooterConstants {
        public static final double THROWER_POWER = 0.5;
    }

    public final class ClimberConstants {
        public static final double CLIMBER_POWER = 0.5;
    }
}
