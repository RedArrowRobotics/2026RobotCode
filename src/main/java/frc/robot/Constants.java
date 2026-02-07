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

    }
    public final class DriveConstants {
        public static final String SWERVE_CONFIG = "swerve-practice";
    }
    public final class FuelShooterConstants {
        public static final double SHOOTER_POWER = 0.5;
    }
    public final class FuelAimingConstants {
        public static final double ROTATOR_MANUAL_POWER = 0.5;
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

        //Rotator Constants
        public static final double ROTATOR_kP = 0.0;
        public static final double ROTATOR_kI = 0.0;
        public static final double ROTATOR_kD = 0.0;
        public static final double ROTATOR_kV = 0.0;
        public static final double ROTATOR_kS = 0.0;
        public static final double ROTATOR_kA = 0.0;
        public static final double ROTATOR_MAX_VELOCITY = 0.0; //RPM
        public static final double ROTATOR_MAX_ACCELERATION = 0.0; //RPM/s
        public static final double ROTATOR_MAX_ERROR = 0.0;
    }
}
