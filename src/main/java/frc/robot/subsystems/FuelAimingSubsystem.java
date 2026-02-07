package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ControlInputs;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FeedforwardConstants;
import frc.robot.Constants.FuelAimingConstants;

public class FuelAimingSubsystem extends SubsystemBase {
	private final SparkMax turretRotator = new SparkMax(DeviceConstants.TURRET_ROTATOR, MotorType.kBrushless);

	public FuelAimingSubsystem() {
		SparkClosedLoopController controller = turretRotator.getClosedLoopController();
		SparkMaxConfig config = new SparkMaxConfig();

		turretRotator.configure(config, ResetMode.kResetSafeParameters,
		PersistMode.kPersistParameters);

		config.closedLoop
		.p(FeedforwardConstants.ROTATOR_kP)
		.i(FeedforwardConstants.ROTATOR_kI)
		.d(FeedforwardConstants.ROTATOR_kD);
		
		config.closedLoop.feedForward
		.kV(FeedforwardConstants.ROTATOR_kV)
		.kS(FeedforwardConstants.ROTATOR_kS)
		.kA(FeedforwardConstants.ROTATOR_kA);

		config.closedLoop.maxMotion
		.cruiseVelocity(FeedforwardConstants.ROTATOR_MAX_VELOCITY)
		.maxAcceleration(FeedforwardConstants.ROTATOR_MAX_ACCELERATION)
		.allowedProfileError(FeedforwardConstants.ROTATOR_MAX_ERROR);

		controller.setSetpoint(0.0, ControlType.kVelocity);
	}

	public Command manualControlCW() {
		return runOnce(() -> {
			turretRotator.set(FuelAimingConstants.ROTATOR_MANUAL_POWER * -1);
		});
	}

	public Command manualControlCCW() {
		return runOnce(() -> {
			turretRotator.set(FuelAimingConstants.ROTATOR_MANUAL_POWER);
		});
	}

	public Command stopRotating() {
		return runOnce(() -> {
			turretRotator.set(0.0);
		});
	}

	public Command automaticAimTowardsHub() {
		return runEnd(() -> {

		}, () -> {
			
		});
	}

	// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);


	// Creates a SysIdRoutine
	SysIdRoutine routine = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism( voltage -> {
				turretRotator.setVoltage(voltage.baseUnitMagnitude());
				},
				// Tell SysId how to record a frame of data for each motor on the mechanism being
				// characterized.
				log -> {
					// Record a frame for the left motors.  Since these share an encoder, we consider
					// the entire group to be one motor.
					log.motor("shooter-1")
						.voltage(
							m_appliedVoltage.mut_replace(
								turretRotator.getBusVoltage() * RobotController.getBatteryVoltage(), Volts))
						.linearPosition(m_distance.mut_replace(turretRotator.getEncoder().getPosition(), Meters))
						.linearVelocity(
							m_velocity.mut_replace(turretRotator.getEncoder().getVelocity(), MetersPerSecond));
						}, this)
	);
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  		return routine.dynamic(direction);
	}
}
