package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FeedforwardConstants;
import frc.robot.Constants.HopperConstants;

public class HopperSubsytem extends SubsystemBase {
	private final SparkFlex hopperExtender = new SparkFlex(DeviceConstants.HOPPER_EXTENDER_MOTOR_ID, MotorType.kBrushless);
	private final SparkClosedLoopController hopperController = hopperExtender.getClosedLoopController();
	private final SparkFlexConfig hopperConfig = new SparkFlexConfig();

	public final Optional<SysId> sysId;

	public HopperSubsytem() {
		if (Constants.DEBUG_ENABLED) {
            sysId = Optional.of(new SysId());
		} else {
            sysId = Optional.empty();
		}

		hopperConfig.closedLoop
		.p(FeedforwardConstants.HOPPER_kP)
		.i(FeedforwardConstants.HOPPER_kI)
		.d(FeedforwardConstants.HOPPER_kD);
		hopperConfig.closedLoop.feedForward
		.kV(FeedforwardConstants.HOPPER_kV)
		.kS(FeedforwardConstants.HOPPER_kS)
		.kA(FeedforwardConstants.HOPPER_kA);
		hopperConfig.closedLoop.maxMotion
		.cruiseVelocity(FeedforwardConstants.HOPPER_MAX_VELOCITY)
		.maxAcceleration(FeedforwardConstants.HOPPER_MAX_ACCELERATION)
		.allowedProfileError(FeedforwardConstants.HOPPER_MAX_ERROR);

		hopperExtender.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		hopperController.setSetpoint(0.0, ControlType.kMAXMotionPositionControl);
	}

	public Command extendHopperManual() {
		return runEnd(() -> {
			if(hopperExtender.getEncoder().getPosition() < HopperConstants.HOPPER_EXTENDED_POSITION) {
				hopperExtender.set(HopperConstants.HOPPER_MANUAL_SPEED);
			} else {
				hopperExtender.set(0.0);
			}
		}, () -> {
			hopperController.setSetpoint(hopperExtender.getEncoder().getPosition(), ControlType.kMAXMotionPositionControl);
			hopperExtender.set(0.0);
		});
	}

	public Command retractHopperManual() {
		return runEnd(() -> {
			if(hopperExtender.getEncoder().getPosition() > HopperConstants.HOPPER_RETRACTED_POSITION) {
				hopperExtender.set(HopperConstants.HOPPER_MANUAL_SPEED * -1);
			} else {
				hopperExtender.set(0.0);
			}
		}, () -> {
			hopperController.setSetpoint(hopperExtender.getEncoder().getPosition(), ControlType.kMAXMotionPositionControl);
			hopperExtender.set(0.0);
		});
	}

	public Command extendHopper() {
		return runOnce(() -> {
			hopperController.setSetpoint(HopperConstants.HOPPER_EXTENDED_POSITION, ControlType.kMAXMotionPositionControl);
		});
	}

	public Command retractHopper() {
		return runOnce(() -> {
			hopperController.setSetpoint(HopperConstants.HOPPER_RETRACTED_POSITION, ControlType.kMAXMotionPositionControl);
		});
	}

	public boolean hopperHome() {
		return hopperController.getSetpoint() == HopperConstants.HOPPER_RETRACTED_POSITION && hopperController.isAtSetpoint();
	}

	public class SysId {
		private final MutVoltage m_appliedVoltage = Volts.mutable(0);
		private final MutDistance m_distance = Meters.mutable(0);
		private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

		private final VelocityUnit<VoltageUnit> voltsPerSecond = Volts.per(Seconds);
		private final Velocity<VoltageUnit> rampRate = voltsPerSecond.of(0.1);
		private final Voltage dynamicVoltage = Volts.of(3.0);
		private final Time runTime = Seconds.of(30.0);

		// Creates a SysIdRoutine
		private final SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, dynamicVoltage, runTime),
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
							}, HopperSubsytem.this)
		);

		public Command quasistatic(SysIdRoutine.Direction direction) {
			return routine.quasistatic(direction);
		}

		public Command dynamic(SysIdRoutine.Direction direction) {
			return routine.dynamic(direction);
		}

		public void configureSendables() {
            var name = HopperSubsytem.this.getName();
            SmartDashboard.putData("SysId/"+name+"/Quasistatic Forward", quasistatic(SysIdRoutine.Direction.kForward));
            SmartDashboard.putData("SysId/"+name+"/Quasistatic Reverse", quasistatic(SysIdRoutine.Direction.kReverse));
            SmartDashboard.putData("SysId/"+name+"/Dynamic Forward", dynamic(SysIdRoutine.Direction.kForward));
            SmartDashboard.putData("SysId/"+name+"/Dynamic Reverse", dynamic(SysIdRoutine.Direction.kReverse));
        }
	}

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		//Telemetry
		builder.addDoubleProperty("Hopper Position", () -> hopperExtender.getEncoder().getPosition(), null);

		sysId.ifPresent(sysid -> sysid.configureSendables());

		//Testing
		SmartDashboard.putData("Extend Hopper Manual", extendHopperManual());
		SmartDashboard.putData("Retract Hopper Manual", retractHopperManual());
		SmartDashboard.putData("Extend Hopper PIDF", extendHopper());
		SmartDashboard.putData("Retract Hopper PIDF", retractHopper());
	}
}
