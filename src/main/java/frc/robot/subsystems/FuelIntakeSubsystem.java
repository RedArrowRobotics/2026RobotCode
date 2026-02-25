package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;

public class FuelIntakeSubsystem extends SubsystemBase {
	private final SparkFlex intakeBar = new SparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax hopperExtender = new SparkMax(IntakeConstants.HOPPER_EXTENDER_MOTOR_ID, MotorType.kBrushless);
	private final SparkClosedLoopController hopperController = hopperExtender.getClosedLoopController();
	private final SparkMaxConfig hopperConfig = new SparkMaxConfig();

	public FuelIntakeSubsystem() {
		hopperExtender.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		hopperConfig.closedLoop
		.p(IntakeConstants.HOPPER_kP)
		.i(IntakeConstants.HOPPER_kI)
		.d(IntakeConstants.HOPPER_kD);
		hopperConfig.closedLoop.feedForward
		.kV(IntakeConstants.HOPPER_kV)
		.kS(IntakeConstants.HOPPER_kS)
		.kA(IntakeConstants.HOPPER_kA);
		hopperConfig.closedLoop.maxMotion
		.cruiseVelocity(IntakeConstants.HOPPER_MAX_VELOCITY)
		.maxAcceleration(IntakeConstants.HOPPER_MAX_ACCELERATION)
		.allowedProfileError(IntakeConstants.HOPPER_MAX_ERROR);
	}

	public Command intakeFuelIn() {
		return runOnce(() -> {
			intakeBar.set(IntakeConstants.INTAKE_SPEED);
		});
	}

	public Command intakeFuelOut() {
		return runOnce(() -> {
			intakeBar.set(IntakeConstants.INTAKE_SPEED * -1);
		});
	}

	public Command intakeStop() {
		return runOnce(() -> {
			intakeBar.set(0.0);
		});
	}

	public Command extendIntake() {
		return runOnce(() -> {
			hopperExtender.set(IntakeConstants.HOPPER_MANUAL_SPEED);
		});
	}

	public Command retractIntake() {
		return runOnce(() -> {
			hopperExtender.set(IntakeConstants.HOPPER_MANUAL_SPEED * -1);
		});
	}

	public Command stopHopperExtension() {
		return runOnce(() -> {
			hopperExtender.set(0.0);
		});
	}

	public Command extendIntakePIDF() {
		return runOnce(() -> {
			hopperController.setSetpoint(IntakeConstants.HOPPER_EXTENDED_POSITION, ControlType.kPosition);
		});
	}

	public Command retractIntakePIDF() {
		return runOnce(() -> {
			hopperController.setSetpoint(IntakeConstants.HOPPER_RETRACTED_POSITION, ControlType.kPosition);
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
		new SysIdRoutine.Mechanism(voltage -> {
				hopperExtender.setVoltage(voltage.baseUnitMagnitude());
				},
				// Tell SysId how to record a frame of data for each motor on the mechanism being
				// characterized.
				log -> {
					// Record a frame for the left motors.  Since these share an encoder, we consider
					// the entire group to be one motor.
					log.motor("hopper-extender")
						.voltage(
							m_appliedVoltage.mut_replace(
								hopperExtender.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
						.linearPosition(m_distance.mut_replace(hopperExtender.getEncoder().getPosition(), Meters))
						.linearVelocity(
							m_velocity.mut_replace(hopperExtender.getEncoder().getVelocity(), MetersPerSecond));
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
		SmartDashboard.putData("Hopper - Quasistatic Foward", sysIdQuasistatic(Direction.kForward));
		SmartDashboard.putData("Hopper - Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
		SmartDashboard.putData("Hopper - Dynamic Foward", sysIdDynamic(Direction.kForward));
		SmartDashboard.putData("Hopper - Dynamic Reverse", sysIdDynamic(Direction.kReverse));

		//Testing
		SmartDashboard.putData("Intake Fuel In", intakeFuelIn());
		SmartDashboard.putData("Intake Fuel Out", intakeFuelOut());
		SmartDashboard.putData("Extend Intake Manual", extendIntake());
		SmartDashboard.putData("Retract Intake Manual", retractIntake());
		SmartDashboard.putData("Extend Intake PIDF", extendIntakePIDF());
		SmartDashboard.putData("Retract Intake PIDF", retractIntakePIDF());
	}
}
