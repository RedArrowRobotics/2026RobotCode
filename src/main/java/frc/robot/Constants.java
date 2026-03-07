package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    /** If set to true, enables SysId and more verbose logging. */
    public static final boolean DEBUG_ENABLED = true;
    
    public final class OperatorConstants {
        public static final int DRIVE_JOYSTICK_PORT = 0;
        public static final int COMPONENTS_BOARD_PORT = 1;
    }

    public final class AprilTagIDs {
        public static final int[] RED_HUB_IDS = {2,3,4,5,8,9,10,11,12};
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

    public final class DeviceConstants {
        public static final int TURRET_ROTATOR = 10;
        public static final int FUEL_SHOOTER_MOTOR_1_ID = 11;
        public static final int FUEL_SHOOTER_MOTOR_2_ID = 12;
        public static final int HOOD_ROTATOR = 18;
        public static final int CLIMBER_MOTOR_ID = 19;
        public static final String LIMELIGHT_FRONT = "limelight-front";
        public static final String LIMELIGHT_BACK = "limelight-back";
        public static final int CLIMBER_ENCODER_CHANNEL = 22;
    }

    public final class DriveConstants {
        public static final String SWERVE_CONFIG = "swerve-practice";
    }

    
    public final class FuelShooterConstants {
        public static final double SHOOTER_RPM = 3250;
        public static final double STOPPED_SPEED = 0.000;
    }

    public final class FuelAimingConstants {
        public static final double TURRET_ROTATOR_MANUAL_POWER = 0.1;
        public static final double HOOD_ROTATOR_MANUAL_POWER = 0.5;
        public static final double STOPPED_SPEED = 0.000;
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
        public static final double TURRET_ROTATOR_kP = 0.00040686;
        public static final double TURRET_ROTATOR_kI = 0.0;
        public static final double TURRET_ROTATOR_kD = 0.0;
        public static final double TURRET_ROTATOR_kV = 0.0010238;
        public static final double TURRET_ROTATOR_kS = 0.11956;
        public static final double TURRET_ROTATOR_kA = 0.000079182;
        public static final double TURRET_ROTATOR_MAX_VELOCITY = 6600 * 60; //RPM
        public static final double TURRET_ROTATOR_MAX_ACCELERATION = 29661 * 60; //RPM/s
        public static final double TURRET_ROTATOR_MAX_ERROR = 1000 * 60; //RPM

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

        //Hopper Constants
        public static final double HOPPER_kP = 0.0;
        public static final double HOPPER_kI = 0.0;
        public static final double HOPPER_kD = 0.0;
        public static final double HOPPER_kV = 0.0;
        public static final double HOPPER_kS = 0.0;
        public static final double HOPPER_kA = 0.0;
        public static final double HOPPER_MAX_VELOCITY = 0.0;
        public static final double HOPPER_MAX_ACCELERATION = 0.0;
        public static final double HOPPER_MAX_ERROR = 0.0;

        //Kicker Constants
        public static final double KICKER_kP = 0.0;
        public static final double KICKER_kI = 0.0;
        public static final double KICKER_kD = 0.0;
        public static final double KICKER_kV = 0.0;
        public static final double KICKER_kS = 0.0;
        public static final double KICKER_kA = 0.0;
        public static final double KICKER_MAX_VELOCITY = 0.0;
        public static final double KICKER_MAX_ACCELERATION = 0.0;
        public static final double KICKER_MAX_ERROR = 0.0;
    }

    public final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 23;
        public static final int HOPPER_EXTENDER_MOTOR_ID = 24;
        public static final double INTAKE_SPEED = 0.5;
        public static final double HOPPER_MANUAL_SPEED = 0.5;

        public static final double HOPPER_EXTENDED_POSITION = 0.0;
        public static final double HOPPER_RETRACTED_POSITION = 0.0;
    }      

    public final class AgitatorConstants {
        public static final int BELT_MOTOR_1_ID = 20;
        public static final int BELT_MOTOR_2_ID = 21;
        public static final int KICKER_MOTOR_ID = 9;

        public static final double BELT_SPEED = 0.5;
        public static final double KICK_RPM = 0.5;
    }
        
    public final class ClimberConstants {
        public static final double CLIMBER_POWER = 0.5;
    }
}
