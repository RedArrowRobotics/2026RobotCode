package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
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

public class FuelAimingSubsystem extends SubsystemBase {
	private final SparkMax turretRotator = new SparkMax(DeviceConstants.TURRET_ROTATOR, MotorType.kBrushless);
	private final SparkClosedLoopController turretController = turretRotator.getClosedLoopController();
	private final SparkMaxConfig turretConfig = new SparkMaxConfig();

	private final SparkMax hoodRotator = new SparkMax(DeviceConstants.HOOD_ROTATOR, MotorType.kBrushed);
	private final SparkClosedLoopController hoodController = hoodRotator.getClosedLoopController();
	private final SparkMaxConfig hoodConfig = new SparkMaxConfig();

	//Important Coordinates
	private final Translation2d hubPosition = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
		case Blue -> FieldPoses.BLUE_HUB;
		case Red -> FieldPoses.RED_HUB;
	};
	private final Translation2d depotTrench = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
		case Blue -> FieldPoses.BLUE_ALLIANCE_TRENCH_DEPOT;
		case Red -> FieldPoses.RED_ALLIANCE_TRENCH_DEPOT;
	};
	private final Translation2d outpostTrench = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
		case Blue -> FieldPoses.BLUE_ALLIANCE_TRENCH_OUTPOST;
		case Red -> FieldPoses.RED_ALLIANCE_TRENCH_OUTPOST;
	};
	private final Translation2d allianceZoneLine = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
		case Blue -> FieldPoses.BLUE_ALLIANCE_LINE;
		case Red -> FieldPoses.RED_ALLIANCE_LINE;
	};
	private boolean inAllianceZone;

	private double thetaToHub;
	private double degreeToHubRelativeToRobot;
	public boolean setpointWithinRange;
	private double distanceToHub;
	private double hoodAngleToHub;

	public FuelAimingSubsystem() {
		//Turret
		turretConfig.closedLoop
		.p(FeedforwardConstants.TURRET_ROTATOR_kP)
		.i(FeedforwardConstants.TURRET_ROTATOR_kI)
		.d(FeedforwardConstants.TURRET_ROTATOR_kD);
		turretConfig.closedLoop.feedForward
		.kV(FeedforwardConstants.TURRET_ROTATOR_kV)
		.kS(FeedforwardConstants.TURRET_ROTATOR_kS)
		.kA(FeedforwardConstants.TURRET_ROTATOR_kA);
		turretConfig.closedLoop.maxMotion
		.cruiseVelocity(FeedforwardConstants.TURRET_ROTATOR_MAX_VELOCITY)
		.maxAcceleration(FeedforwardConstants.TURRET_ROTATOR_MAX_ACCELERATION)
		.allowedProfileError(FeedforwardConstants.TURRET_ROTATOR_MAX_ERROR);

		turretRotator.configure(turretConfig, ResetMode.kResetSafeParameters,
		PersistMode.kPersistParameters);

		turretController.setSetpoint(0.0, ControlType.kMAXMotionPositionControl);

		//Hood
		hoodConfig.closedLoop
		.p(FeedforwardConstants.HOOD_ROTATOR_kP)
		.i(FeedforwardConstants.HOOD_ROTATOR_kI)
		.d(FeedforwardConstants.HOOD_ROTATOR_kD);
		hoodConfig.closedLoop.feedForward
		.kV(FeedforwardConstants.HOOD_ROTATOR_kV)
		.kS(FeedforwardConstants.HOOD_ROTATOR_kS)
		.kA(FeedforwardConstants.HOOD_ROTATOR_kA);
		hoodConfig.closedLoop.maxMotion
		.cruiseVelocity(FeedforwardConstants.HOOD_ROTATOR_MAX_VELOCITY)
		.maxAcceleration(FeedforwardConstants.HOOD_ROTATOR_MAX_ACCELERATION)
		.allowedProfileError(FeedforwardConstants.HOOD_ROTATOR_MAX_ERROR);

		hoodConfig.inverted(true);
		hoodConfig.encoder.inverted(true);

		hoodRotator.configure(hoodConfig, ResetMode.kResetSafeParameters,
		PersistMode.kPersistParameters);

		hoodController.setSetpoint(0.0, ControlType.kMAXMotionPositionControl);
	}

	public Command manualTurretControlCW() {
		return runOnce(() -> {
			turretRotator.set(FuelAimingConstants.TURRET_ROTATOR_MANUAL_POWER * -1);
			System.out.println("Set turret to :"+turretRotator.get());
		});
	}

	public Command manualTurretControlCCW() {
		return runOnce(() -> {
			turretRotator.set(FuelAimingConstants.TURRET_ROTATOR_MANUAL_POWER);
			System.out.println("Set turret to :"+turretRotator.get());
		});
	}

	public Command turretToPosition(double position) {
		return runOnce(() -> {
			turretController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
		});
	}

	public Command manualTurretControlStop() {
		return runOnce(() -> {
			turretRotator.set(FuelAimingConstants.STOPPED_SPEED);
			System.out.println("Stop turret to :"+turretRotator.get());
		});
	}

	public Command manualHoodControlUp() {
		return runOnce(() -> {
			hoodRotator.set(FuelAimingConstants.HOOD_ROTATOR_MANUAL_POWER);
		});
	}

	public Command manualHoodControlDown() {
		return runOnce(() -> {
			hoodRotator.set(FuelAimingConstants.HOOD_ROTATOR_MANUAL_POWER * -1);
		});
	}

	public Command hoodToPosition(double position) {
		return runOnce(() -> {
			hoodController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
		});
	}

	public Command manualHoodControlStop() {
		return runOnce(() -> {
			hoodRotator.set(FuelAimingConstants.STOPPED_SPEED);
		});
	}

	public Command automaticAimRoutine(Supplier<Pose2d> robotPose) {
		return run(() -> {
			//Turret Control
			thetaToHub = Math.atan((hubPosition.getY() - robotPose.get().getY()) /
									 (hubPosition.getX() - robotPose.get().getX()));
			degreeToHubRelativeToRobot = (thetaToHub * (180/Math.PI)) - robotPose.get().getRotation().getDegrees(); /*Convert from radians to degrees and subtract yaw of the robot*/
			inAllianceZone = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
				case Blue -> robotPose.get().getX() < allianceZoneLine.getX();
				case Red -> robotPose.get().getX() > allianceZoneLine.getX();
			};
			if(degreeToHubRelativeToRobot > 90) {
				degreeToHubRelativeToRobot  = 90;
				setpointWithinRange = false;
			} else if(degreeToHubRelativeToRobot < -90) {
				degreeToHubRelativeToRobot = -90;
				setpointWithinRange = false;
			} else {
				setpointWithinRange = true;
			}
			
			//TODO: restrain degreeRelativeToRobot to bounds that the turret can physically reach
			if(inAllianceZone) {
				turretController.setSetpoint(degreeToHubRelativeToRobot * -0.2132, ControlType.kMAXMotionPositionControl);
			} else {
				//turretController.setSetpoint(robotPose.get().getRotation().getDegrees() * 0.2132, ControlType.kMAXMotionPositionControl);
			}
						
			//Hood Control
			//Do math to figure out optimal hood angle as a function of distance
			//Min Distance: 30 in -> 1.359m     Max Distance: 241.7 in -> 6.139m
			//Min Angle: 60 deg       Max Angle: 80 deg
			//-4.184 is slope
			distanceToHub = hubPosition.getDistance(robotPose.get().getTranslation());
			hoodAngleToHub = 80 - 4.184 * (distanceToHub - 1.359);
			//Convert angle to encoder counts (gear ratio of 420:25 = 16.8)
			if(robotPose.get().getTranslation().getDistance(outpostTrench) > Math.abs(allianceZoneLine.getX() - outpostTrench.getX()) ||
			   robotPose.get().getTranslation().getDistance(depotTrench) > Math.abs(allianceZoneLine.getX() - depotTrench.getX()) ) {
				//hoodController.setSetpoint((hoodAngleToHub * 16.8) / 360, ControlType.kMaxMotionPositionControl);
			   } else {
				//hoodController.setSetpoint(0.0, ControlType.kMAXMotionPositionControl);
			   }
		});
	}

	//Todo: Add sprinkler routine to turret :l

	// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
	private final MutVoltage m_appliedVoltage = Volts.mutable(0);
	// Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
	private final MutAngle m_distance = Rotations.mutable(0);
	// Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
	private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);


	// Creates a SysIdRoutine
	SysIdRoutine turretRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(voltage -> {
				turretRotator.setVoltage(voltage.baseUnitMagnitude());
				},
				// Tell SysId how to record a frame of data for each motor on the mechanism being
				// characterized.
				log -> {
					// Record a frame for the left motors.  Since these share an encoder, we consider
					// the entire group to be one motor.
					log.motor("turret-rotator")
						.voltage(
							m_appliedVoltage.mut_replace(
								turretRotator.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
						.angularPosition(m_distance.mut_replace(turretRotator.getEncoder().getPosition(), Rotations))
						.angularVelocity(
							m_velocity.mut_replace(turretRotator.getEncoder().getVelocity(), RotationsPerSecond));
						}, this)
	);

	public Command sysIdQuasistaticTurret(SysIdRoutine.Direction direction) {
		return turretRoutine.quasistatic(direction);
	}

	public Command sysIdDynamicTurret(SysIdRoutine.Direction direction) {
		return turretRoutine.dynamic(direction);
	}

	private final VelocityUnit<VoltageUnit> voltsPerSeconds = Volts.per(Seconds);
	private final Velocity<VoltageUnit> rampRate = voltsPerSeconds.of(1.0);

	SysIdRoutine hoodRoutine = new SysIdRoutine(
	 	new SysIdRoutine.Config(rampRate, Volts.of(5.0), Seconds.of(10.0)),
	 	new SysIdRoutine.Mechanism(voltage -> {
	 				hoodRotator.setVoltage(voltage.baseUnitMagnitude());
	 			},
	 			// Tell SysId how to record a frame of data for each motor on the mechanism being
	 			// characterized.
	 			log -> {
	 				// Record a frame for the left motors.  Since these share an encoder, we consider
	 				// the entire group to be one motor.
	 				log.motor("hood-rotator")
	 					.voltage(
	 						m_appliedVoltage.mut_replace(
	 							hoodRotator.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
	 					.angularPosition(m_distance.mut_replace(hoodRotator.getEncoder().getPosition(), Rotations))
	 					.angularVelocity(
	 						m_velocity.mut_replace(hoodRotator.getEncoder().getVelocity(), RotationsPerSecond));
	 					}, this)
	);

	 public Command sysIdQuasistaticHood(SysIdRoutine.Direction direction) {
  	 	return hoodRoutine.quasistatic(direction);
	 }

	 public Command sysIdDynamicHood(SysIdRoutine.Direction direction) {
  	 	return hoodRoutine.dynamic(direction);
	 }

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		//Telemetry
		builder.addDoubleProperty("Turret Encoder", () -> turretRotator.getEncoder().getPosition(), null);
		builder.addDoubleProperty("Turret Power", () -> turretRotator.get(), null);
		builder.addDoubleProperty("Turret Voltage", () -> turretRotator.getAppliedOutput(), null);
		builder.addDoubleProperty("Turret Setpoint", () -> turretController.getSetpoint(), null);
		builder.addDoubleProperty("Turret Velocity", () -> turretRotator.getEncoder().getVelocity(), null);

		builder.addDoubleProperty("Hood Power", () -> hoodRotator.get(), null);
		builder.addDoubleProperty("Hood Voltage", () -> hoodRotator.getAppliedOutput(), null);
		builder.addDoubleProperty("Hood Setpoint", () -> hoodController.getSetpoint(), null);
		builder.addDoubleProperty("Hood Encoder", () -> hoodRotator.getEncoder().getPosition(), null);
		builder.addDoubleProperty("Hood Velocity", () -> hoodRotator.getEncoder().getVelocity(), null);

		builder.addDoubleProperty("Theta to Hub", () -> thetaToHub, null);
		builder.addDoubleProperty("Degrees Relative to Robot", () -> degreeToHubRelativeToRobot, null);
		builder.addBooleanProperty("In Alliance Zone", () -> inAllianceZone, null);
		builder.addDoubleProperty("Distance from Hub", () -> distanceToHub, null);

		//Sys ID
		SmartDashboard.putData("Turret - Run Forward Dynamic", sysIdDynamicTurret(Direction.kForward));
		SmartDashboard.putData("Turret - Run Reverse Dynamic", sysIdDynamicTurret(Direction.kReverse));
		SmartDashboard.putData("Turret - Run Forward Quasistatic", sysIdQuasistaticTurret(Direction.kForward));
		SmartDashboard.putData("Turret - Run Reverse Quasistatic", sysIdQuasistaticTurret(Direction.kReverse));
		SmartDashboard.putData("Hood - Run Forward Dynamic", sysIdDynamicHood(Direction.kForward));
		SmartDashboard.putData("Hood - Run Reverse Dynamic", sysIdDynamicHood(Direction.kReverse));
		SmartDashboard.putData("Hood - Run Forward Quasistatic", sysIdQuasistaticHood(Direction.kForward));
		SmartDashboard.putData("Hood - Run Reverse Quasistatic", sysIdQuasistaticHood(Direction.kReverse));

		//Testing
		SmartDashboard.putData("Manual Turret CW", manualTurretControlCW());
		SmartDashboard.putData("Manual Turret CCW", manualTurretControlCCW());
		SmartDashboard.putData("Manual Turret Stop", manualTurretControlStop());
		SmartDashboard.putData("Manual Turret Run CCW", turretToPosition(100.0));
		SmartDashboard.putData("Manual Turret Run CW", turretToPosition(0.0));

		SmartDashboard.putData("Manual Hood Up", manualHoodControlUp());
		SmartDashboard.putData("Manual Hood Down", manualHoodControlDown());
		SmartDashboard.putData("Manual Hood Stop", manualHoodControlStop());
		SmartDashboard.putData("Manual Hood Run Up", hoodToPosition(2.0));
		SmartDashboard.putData("Manual Hood Run Down", hoodToPosition(0.0));
	}
}
