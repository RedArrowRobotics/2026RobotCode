package frc.robot;

public final class Constants {
    public final class OperatorConstants {
        public static final int DRIVE_JOYSTICK_PORT = 0;
        public static final int COMPONENTS_BOARD_PORT = 1;
    }
    public final class DeviceConstants {
        public static final int FUEL_SHOOTER_MOTOR_1_ID = 15;
        public static final int FUEL_SHOOTER_MOTOR_2_ID = 16;
        public static final int TURRET_ROTATOR = 17;
        public static final int HOOD_ROTATOR = 18;

    }
    public final class DriveConstants {
        public static final String SWERVE_CONFIG = "swerve-practice";
    }
    public final class FuelShooterConstants {
        public static final double SHOOTER_POWER = 0.5;
    }
    public final class FuelAimingConstants {
        public static final double TURRET_ROTATOR_MANUAL_POWER = 0.5;
        public static final double HOOD_ROTATOR_MANUAL_POWER = 0.5;
        public static final double STOPPED_SPEED = 0.00;
    }
    public final class FeedforwardConstants {
        //Shooter Constants
        public static final double SHOOTER_kP_NO_FUEL = 0.0;
        public static final double SHOOTER_kI_NO_FUEL = 0.0;
        public static final double SHOOTER_kD_NO_FUEL = 0.0;
        public static final double SHOOTER_kP_FUEL = 0.0;
        public static final double SHOOTER_kI_FUEL = 0.0;
        public static final double SHOOTER_kD_FUEL = 0.0;
        public static final double SHOOTER_kV = 0.0;
        public static final double SHOOTER_kS = 0.0;
        public static final double SHOOTER_kA = 0.0;
        public static final double SHOOTER_MAX_VELOCTIY = 0.0; //RPM
        public static final double SHOOTER_MAX_ACCELERATION = 0.0; //RPM/s
        public static final double SHOOTER_MAX_ERROR = 0.0;

        //Turret Constants
        public static final double TURRET_ROTATOR_kP = 0.0;
        public static final double TURRET_ROTATOR_kI = 0.0;
        public static final double TURRET_ROTATOR_kD = 0.0;
        public static final double TURRET_ROTATOR_kV = 0.0;
        public static final double TURRET_ROTATOR_kS = 0.0;
        public static final double TURRET_ROTATOR_kA = 0.0;
        public static final double TURRET_ROTATOR_MAX_VELOCITY = 0.0; //RPM
        public static final double TURRET_ROTATOR_MAX_ACCELERATION = 0.0; //RPM/s
        public static final double TURRET_ROTATOR_MAX_ERROR = 0.0;

        //Hood Constants
        public static final double HOOD_ROTATOR_kP = 0.0;
        public static final double HOOD_ROTATOR_kI = 0.0;
        public static final double HOOD_ROTATOR_kD = 0.0;
        public static final double HOOD_ROTATOR_kV = 0.0;
        public static final double HOOD_ROTATOR_kS = 0.0;
        public static final double HOOD_ROTATOR_kA = 0.0;
        public static final double HOOD_ROTATOR_MAX_VELOCITY = 0.0; //RPM
        public static final double HOOD_ROTATOR_MAX_ACCELERATION = 0.0; //RPM/s
        public static final double HOOD_ROTATOR_MAX_ERROR = 0.0;
    }
}
