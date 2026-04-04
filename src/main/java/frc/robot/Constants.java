package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    /** If set to true, enables SysId and more verbose logging. */
    public static final boolean DEBUG_ENABLED = true;
    
    public final class OperatorConstants {
        public static final int DRIVE_JOYSTICK_PORT = 0;
        public static final int COMPONENTS_BOARD_1_PORT = 1;
        public static final int COMPONENTS_BOARD_2_PORT = 2;
        public static final int EMERGENCY_JANKY_FLIGHT_STICK = 3;
    }

    public final class InputConstants {
        //Board 1
        public static final int SHOOT_FUEL = 1;
        public static final int INTAKE_OUT = 2;
        public static final int INTAKE_IN = 3;
        public static final int EXTEND_HOPPER = 4;
        public static final int RETRACT_HOPPER = 5;
        public static final int CLIMBER_UP = 6;
        public static final int CLIMBER_DOWN = 7;
        public static final int DRIVE_ORIENTATION_SWITCH = 12;

        //Board 2
        public static final int MANUAL_SWITCH = 1;
        public static final int MANUAL_HOOD_UP = 2;
        public static final int MANUAL_HOOD_DOWN = 3;
        public static final int MANUAL_TURRET_CW = 4;
        public static final int MANUAL_TURRET_CCW = 5;
    }

    public final class DeviceConstants {
        //Motor Controllers
        public static final int KICKER_MOTOR_ID = 10;
        public static final int FUEL_SHOOTER_MOTOR_1_ID = 11;
        public static final int FUEL_SHOOTER_MOTOR_2_ID = 12;
        public static final int TURRET_ROTATOR = 13;
        public static final int SPINNER_MOTOR_ID = 14;
        public static final int BELT_MOTOR_ID = 15;
        public static final int HOOD_ROTATOR = 17;
        public static final int CLIMBER_MOTOR_ID = 19;
        public static final int INTAKE_MOTOR_ID = 23;
        public static final int HOPPER_EXTENDER_MOTOR_ID = 24;

        //Limelights
        public static final String LIMELIGHT_FRONT = "limelight-front";
        public static final String LIMELIGHT_BACK = "limelight-back";

        //External Encoders/Sensors
        public static final int CLIMBER_ENCODER_CHANNEL = 22;
        public static final int TURRET_AIMMER_LIMIT_SWITCH_CHANNEL = 9;
    }

    public final class AprilTagIDs {
        public static final int[] RED_HUB_IDS = {2,3,4,5,8,9,10,11};
        public static final int[] RED_TRENCH_IDS = {1,6,7,12};
        public static final int[] RED_OUTPOST_IDS = {13,14};
        public static final int[] RED_TOWER_IDS = {15,16};

        public static final int[] BLUE_HUB_IDS = {18,19,20,21,24,25,26,27};
        public static final int[] BLUE_TRENCH_IDS = {17,22,23,28};
        public static final int[] BLUE_OUTPOST_IDS = {29,30};
        public static final int[] BLUE_TOWER_IDS = {31,32};
    }

    public final class FieldPoses {
        public static final Translation2d RED_HUB = new Translation2d(11.905, 4.075);
        public static final Translation2d BLUE_HUB = new Translation2d(4.645, 4.075);

        public static final Translation2d RED_ALLIANCE_LINE = new Translation2d(12.500, 0.000);
        public static final Translation2d BLUE_ALLIANCE_LINE = new Translation2d(4.000, 0.000);

        public static final Translation2d RED_ALLIANCE_TRENCH_DEPOT = new Translation2d(11.900, 0.650);
        public static final Translation2d RED_ALLIANCE_TRENCH_OUTPOST = new Translation2d(11.900, 7.500);
        public static final Translation2d BLUE_ALLIANCE_TRENCH_DEPOT = new Translation2d(4.700, 7.500);
        public static final Translation2d BLUE_ALLIANCE_TRENCH_OUTPOST = new Translation2d(4.700, 0.650);
    }

    public final class FeedforwardConstants {
        //Shooter Constants
        public static final double SHOOTER_kP_NO_FUEL = 0.00027106;
        public static final double SHOOTER_kI_NO_FUEL = 0.0;
        public static final double SHOOTER_kD_NO_FUEL = 0.0;
        public static final double SHOOTER_kP_FUEL = 0.0;
        public static final double SHOOTER_kI_FUEL = 0.0;
        public static final double SHOOTER_kD_FUEL = 0.0;
        public static final double SHOOTER_kV = 0.0018191;
        public static final double SHOOTER_kS = 0.04035;
        public static final double SHOOTER_kA = 0.00023685;
        public static final double SHOOTER_MAX_VELOCTIY = 6500; //RPM
        public static final double SHOOTER_MAX_ACCELERATION = 7500; //RPM/s
        public static final double SHOOTER_MAX_ERROR = 50.0;

        //Turret Constants
        public static final double TURRET_ROTATOR_kP = 0.45558;
        public static final double TURRET_ROTATOR_kI = 0.0;
        public static final double TURRET_ROTATOR_kD = 0.0080607;
        public static final double TURRET_ROTATOR_kV = 0.001059;
        public static final double TURRET_ROTATOR_kS = 0.23111;
        public static final double TURRET_ROTATOR_kA = 8.407E-05;
        public static final double TURRET_ROTATOR_MAX_VELOCITY = 8000; //RPM
        public static final double TURRET_ROTATOR_MAX_ACCELERATION = 12000; //RPM/s
        public static final double TURRET_ROTATOR_MAX_ERROR = 100; //RPM

        //Hood Constants
        public static final double HOOD_ROTATOR_kP = 14.042;
        public static final double HOOD_ROTATOR_kI = 0.0;
        public static final double HOOD_ROTATOR_kD = 0.13104;
        public static final double HOOD_ROTATOR_kV = 0.033071;
        public static final double HOOD_ROTATOR_kS = 0.85644;
        public static final double HOOD_ROTATOR_kA = 0.0031839;
        public static final double HOOD_ROTATOR_MAX_VELOCITY = 165.0; //RPM
        public static final double HOOD_ROTATOR_MAX_ACCELERATION = 24.621 * 60; //RPM/s
        public static final double HOOD_ROTATOR_MAX_ERROR = 84.241;

        //Hopper Constants
        public static final double HOPPER_kP = 3.596;
        public static final double HOPPER_kI = 0.0;
        public static final double HOPPER_kD = 0.0;
        public static final double HOPPER_kV = 0.025052;
        public static final double HOPPER_kS = 0.55954;
        public static final double HOPPER_kA = 0.012042;
        public static final Double HOPPER_kG = 0.15691;
        public static final double HOPPER_MAX_VELOCITY = 300;
        public static final double HOPPER_MAX_ACCELERATION = 300;
        public static final double HOPPER_MAX_ERROR = 50;
        
        //Kicker Constants
        public static final double KICKER_kP = 0.00014281;
        public static final double KICKER_kI = 0.0;
        public static final double KICKER_kD = 0.0;
        public static final double KICKER_kV = 0.0020888;
        public static final double KICKER_kS = 0.34742;
        public static final double KICKER_kA = 0.00020057;
        public static final double KICKER_MAX_VELOCITY = 5000;
        public static final double KICKER_MAX_ACCELERATION = 6335;
        public static final double KICKER_MAX_ERROR = 75.0;

        //Climber Constants
        public static final double CLIMBER_kP = 1.5061;
        public static final double CLIMBER_kI = 0.0;
        public static final double CLIMBER_kD = 0.016293;
        public static final double CLIMBER_kV = 0.0020142;
        public static final double CLIMBER_kS = 0.086225;
        public static final double CLIMBER_kA = 0.00020083;
        public static final double CLIMBER_kG = 0.022574;
        public static final double CLIMBER_MAX_VELOCITY = 3500;
        public static final double CLIMBER_MAX_ACCELERATION = 10000;
        public static final double CLIMBER_MAX_ERROR = 1000;
    }

    public final class AgitatorConstants {
        public static final double BELT_SPEED = 1.0;
        public static final double KICK_SPEED = 0.5;
        public static final double KICK_RPM = 4000;
        public static final double SPIN_SPEED = 1.0;
    }
    
    public final class ClimberConstants {
        public static final double CLIMBER_POWER = 0.8;

        public static final double CLIMBER_DOWN_POSITION = 0.0;
        public static final double CLIMBER_UP_POSITION = 170.0;
    }

    public final class DriveConstants {
        public static final String SWERVE_CONFIG = "swerve";
    }

    public final class FuelAimingConstants {
        //Turret
        public static final double TURRET_ROTATOR_MANUAL_POWER = 0.1;
        public static final double TURRET_ROTATION_ZERO = 0.000;
        public static final double DEGREES_TO_ROTATIONS = 0.2132;
        public static final double STOPPED_SPEED = 0.000;
        public static final double TURRET_OFFSET_SIZE = 5.0;
       
        //Hood
        public static final double HOOD_DOWN_POSITION = 0.0;
        public static final double HOOD_UP_POSITION = 2.142;
        public static final double HOOD_ROTATOR_MANUAL_POWER = 1.0;
    }

    public final class FuelShooterConstants {
        public static final double SHOOTER_RPM = 3250;
        public static final double STOPPED_SPEED = 0.000;
        public static final double SHOOTER_OFFSET_SIZE = 0.1;
    }

    public final class HopperConstants {
        public static final double HOPPER_MANUAL_SPEED = 0.5;

        public static final double HOPPER_EXTENDED_POSITION = 1.72;
        public static final double HOPPER_RETRACTED_POSITION = 0.0;
    }

    public final class IntakeConstants {
        public static final double INTAKE_SPEED = 1.0;
    }
}
    