package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FeedforwardConstants;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FuelAimingConstants;
import frc.robot.Constants.FuelShooterConstants;
import frc.robot.LimelightHelpers;

public class FuelShooterSubsystem extends SubsystemBase {
	private final SparkFlex shooterMotor1 = new SparkFlex(DeviceConstants.FUEL_SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
	private final SparkFlex shooterMotor2 = new SparkFlex(DeviceConstants.FUEL_SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
	private final SparkClosedLoopController controller = shooterMotor1.getClosedLoopController();
	private final SparkFlexConfig motor1Config = new SparkFlexConfig();
	private final SparkFlexConfig motor2Config = new SparkFlexConfig();
	private ShooterStates state = ShooterStates.AT_SPEED;
	private final Translation2d hubPosition = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
                case Blue -> FieldPoses.BLUE_HUB;
                case Red -> FieldPoses.RED_HUB;
            };
	
	public FuelShooterSubsystem () {
		motor2Config.follow(DeviceConstants.FUEL_SHOOTER_MOTOR_1_ID, true);

		shooterMotor1.configure(motor1Config, ResetMode.kResetSafeParameters,
		PersistMode.kPersistParameters);
		shooterMotor2.configure(motor2Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

		motor1Config.closedLoop
		.p(FeedforwardConstants.SHOOTER_kP_NO_FUEL)
		.i(FeedforwardConstants.SHOOTER_kI_NO_FUEL)
		.d(FeedforwardConstants.SHOOTER_kD_NO_FUEL)
		//Secondary Values
		.p(FeedforwardConstants.SHOOTER_kP_FUEL, ClosedLoopSlot.kSlot1)
		.i(FeedforwardConstants.SHOOTER_kI_FUEL, ClosedLoopSlot.kSlot1)
		.d(FeedforwardConstants.SHOOTER_kD_FUEL, ClosedLoopSlot.kSlot1);
		motor1Config.closedLoop.feedForward
		.kV(FeedforwardConstants.SHOOTER_kV)
		.kS(FeedforwardConstants.SHOOTER_kS)
		.kA(FeedforwardConstants.SHOOTER_kA);
		motor1Config.closedLoop.maxMotion
		.cruiseVelocity(FeedforwardConstants.SHOOTER_MAX_VELOCTIY)
		.maxAcceleration(FeedforwardConstants.SHOOTER_MAX_ACCELERATION)
		.allowedProfileError(FeedforwardConstants.SHOOTER_MAX_ERROR);

		controller.setSetpoint(0.0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
	}

	public enum ShooterStates {
		ACCELERATING,
		AT_SPEED;
	}

	public Command shootFuelVarSpeed(Supplier<Pose2d> robotPose) {
		return run(() -> {
			double distance = hubPosition.getDistance(robotPose.get().getTranslation());
			//Do math to figure out optimal motor speed as a function of distance
			//Min Distance: 30 in -> 1.359m     Max Distance: 241.7 in -> 6.139
			//Min Angle: 60 deg       Max Angle: 80 deg
			//Min RPM: 2200 rpm       Max RPM: 3100 rpm
			//slope is 188.285
			double speed = 2200 + 188.285 * (distance - 1.359);
			controller.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
		});
	}

	public Command shootFuel() {
		return runOnce(() -> {
			controller.setSetpoint(FuelShooterConstants.SHOOTER_POWER, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
		});
	}

	public Command shooterDeactivate() {
		return runOnce(() -> {
			controller.setSetpoint(FuelShooterConstants.STOPPED_SPEED, ControlType.kVelocity);
		});
	}

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_distance = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);


		// Creates a SysIdRoutine
	SysIdRoutine routine = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism( voltage -> {
				shooterMotor1.setVoltage(voltage.baseUnitMagnitude());
				},
				// Tell SysId how to record a frame of data for each motor on the mechanism being
				// characterized.
				log -> {
					// Record a frame for the left motors.  Since these share an encoder, we consider
					// the entire group to be one motor.
					log.motor("shooter-1")
						.voltage(
							m_appliedVoltage.mut_replace(
								shooterMotor1.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
						.angularPosition(m_distance.mut_replace(shooterMotor1.getEncoder().getPosition(), Rotations))
						.angularVelocity(
							m_velocity.mut_replace(shooterMotor1.getEncoder().getVelocity(), RotationsPerSecond));
						}, this)
	);

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		//Sys ID
		SmartDashboard.putData("Shooter - Run Forward Dynamic", sysIdDynamic(Direction.kForward));
		SmartDashboard.putData("Shooter - Run Reverse Dynamic", sysIdDynamic(Direction.kReverse));
		SmartDashboard.putData("Shooter - Run Forward Quasistatic", sysIdQuasistatic(Direction.kForward));
		SmartDashboard.putData("Shooter - Run Reverse Quasistatic", sysIdQuasistatic(Direction.kReverse));

		//Testing
		SmartDashboard.putData("Shoot Fuel", shootFuel());
		SmartDashboard.putData("Shooter Deactivate", shooterDeactivate());
	}
}
