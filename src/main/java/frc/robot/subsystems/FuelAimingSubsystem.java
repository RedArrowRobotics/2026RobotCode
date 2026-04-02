package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FeedforwardConstants;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FuelAimingConstants;

public class FuelAimingSubsystem extends SubsystemBase {
	private final SparkMax turretRotator = new SparkMax(DeviceConstants.TURRET_ROTATOR, MotorType.kBrushless);
	private final SparkMax hoodRotator = new SparkMax(DeviceConstants.HOOD_ROTATOR, MotorType.kBrushed);
	private final SparkMaxConfig turretConfig = new SparkMaxConfig();
	private final SparkMaxConfig hoodConfig = new SparkMaxConfig();
	private final SparkClosedLoopController turretController = turretRotator.getClosedLoopController();
	private final SparkClosedLoopController hoodController = hoodRotator.getClosedLoopController();
	private DigitalInput turretAimingLimitSwitch = new DigitalInput(DeviceConstants.TURRET_AIMMER_LIMIT_SWITCH_CHANNEL);
	private boolean turretSetZeroStart = false;

	private boolean inAllianceZone;
	private double distanceToHub;
	private double thetaToHub;
	private double degreeToHubRelativeToRobot;
	private double hoodEncoderPosition;
	private boolean turretSetpointWithinRange;
	private boolean hoodSetpointWithinRange;
	
	public final Optional<SysIdTurret> sysIdTurret;
    public final Optional<SysIdHood> sysIdHood;

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

	public FuelAimingSubsystem() {
		if (Constants.DEBUG_ENABLED) {
            sysIdTurret = Optional.of(new SysIdTurret());
            sysIdHood = Optional.of(new SysIdHood());
		} else {
            sysIdTurret = Optional.empty();
            sysIdHood = Optional.empty();
		}

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

		turretConfig.smartCurrentLimit(20);
		turretConfig.inverted(true);

		turretRotator.configure(turretConfig, ResetMode.kResetSafeParameters,
		PersistMode.kPersistParameters);

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
		hoodConfig.idleMode(IdleMode.kCoast);

		hoodRotator.configure(hoodConfig, ResetMode.kResetSafeParameters,
		PersistMode.kPersistParameters);
	}

	public Command timeOut() {
		return runOnce(() -> {
			turretRotator.set(0.0);
			hoodRotator.set(0.0);
		});
	}

	public Command zeroTurret() {
		return runEnd(() -> {
			turretRotator.set(0.1);
		}, () -> {
			turretRotator.set(0.0);
			turretRotator.getEncoder().setPosition(Constants.FuelAimingConstants.TURRET_ROTATION_ZERO * Constants.FuelAimingConstants.DEGREES_TO_ROTATIONS);
			turretSetZeroStart = true;
		}).onlyWhile(() -> !turretAimingLimitSwitch.get()).onlyIf(() -> !turretSetZeroStart);
	}

	public Command manualTurretControlCW() {
		return startEnd(() -> {
			turretRotator.set(FuelAimingConstants.TURRET_ROTATOR_MANUAL_POWER * -1);
			System.out.println("Set turret to :"+turretRotator.get());
		}, () -> {
			turretController.setSetpoint(turretRotator.getEncoder().getPosition(), ControlType.kMAXMotionPositionControl);
			turretRotator.set(0.0);
		});
	}

	public Command manualTurretControlCCW() {
		return startEnd(() -> {
			turretRotator.set(FuelAimingConstants.TURRET_ROTATOR_MANUAL_POWER);
			System.out.println("Set turret to :"+turretRotator.get());
		}, () -> {
			turretController.setSetpoint(turretRotator.getEncoder().getPosition(), ControlType.kMAXMotionPositionControl);
			turretRotator.set(0.0);
		});
	}

	public Command manualHoodControlUp() {
		return runEnd(() -> {
			hoodRotator.set(FuelAimingConstants.HOOD_ROTATOR_MANUAL_POWER);
		}, () -> {
			hoodController.setSetpoint(hoodRotator.getEncoder().getPosition(), ControlType.kMAXMotionPositionControl);
			hoodRotator.set(0.0);
		});
	}

	public Command manualHoodControlDown() {
		return startEnd(() -> {
			hoodRotator.set(FuelAimingConstants.HOOD_ROTATOR_MANUAL_POWER * -1);
		}, () -> {
			hoodController.setSetpoint(hoodRotator.getEncoder().getPosition(), ControlType.kMAXMotionPositionControl); 
			hoodRotator.set(0.0);
		});
	}

	public Command automaticAimRoutine(Supplier<Pose2d> robotPose) {
		return run(() -> {
			inAllianceZone = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
				case Blue -> robotPose.get().getX() < allianceZoneLine.getX();
				case Red -> robotPose.get().getX() > allianceZoneLine.getX();
			};
			//Turret Control
			thetaToHub = Math.atan((hubPosition.getY() - robotPose.get().getY()) /
								   (hubPosition.getX() - robotPose.get().getX()));
			degreeToHubRelativeToRobot = (thetaToHub * (180/Math.PI)) - robotPose.get().getRotation().getDegrees(); /*Convert from radians to degrees and subtract yaw of the robot*/
			if(degreeToHubRelativeToRobot > 90) {
				degreeToHubRelativeToRobot  = 90;
				turretSetpointWithinRange = false;
			} else if(degreeToHubRelativeToRobot < -90) {
				degreeToHubRelativeToRobot = -90;
				turretSetpointWithinRange = false;
			} else {
				turretSetpointWithinRange = true;
			}
			if(inAllianceZone && turretSetpointWithinRange) {
				turretController.setSetpoint(degreeToHubRelativeToRobot * FuelAimingConstants.DEGREES_TO_ROTATIONS, ControlType.kMAXMotionPositionControl);
			} else {
				turretRotator.set(0.0);
			}
			
			//Hood Control
			//Do math to figure out optimal hood angle as a function of distance
			//Min Distance: 30 in -> 1.359m     Max Distance: 241.7 in -> 6.139m
			distanceToHub = hubPosition.getDistance(robotPose.get().getTranslation());
			hoodEncoderPosition = 1.4 + 0.01052 * (distanceToHub - 241);
			if(hoodEncoderPosition < 0) {
				hoodEncoderPosition = 0;
				hoodSetpointWithinRange = false;
			} else if(hoodEncoderPosition > 2.0) {
				hoodEncoderPosition = 2.0;
				hoodSetpointWithinRange = false;
			} else {
				hoodSetpointWithinRange = true;
			}
			if(robotPose.get().getTranslation().getDistance(outpostTrench) - 0.6 > Math.abs(allianceZoneLine.getX() - outpostTrench.getX()) && inAllianceZone
				||
			   robotPose.get().getTranslation().getDistance(depotTrench) - 0.6 > Math.abs(allianceZoneLine.getX() - depotTrench.getX()) && inAllianceZone) {
				//hoodController.setSetpoint(hoodEncoderPosition, ControlType.kMAXMotionPositionControl);
			   } else {
				//hoodController.setSetpoint(0.0, ControlType.kMAXMotionPositionControl);
			   }
		});
	}

	public boolean turretAtSetpoint() {
		return turretController.isAtSetpoint();
	}

	//Todo: Add sprinkler routine to turret :l

	public class SysIdTurret {
        private final MutVoltage appliedVoltage = Volts.mutable(0);
        private final MutAngle distance = Rotations.mutable(0);
        private final MutAngularVelocity velocity = RotationsPerSecond.mutable(0);
        private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Drive motor controllers
                        voltage -> {
                            turretRotator.setVoltage(voltage.baseUnitMagnitude());
						},
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being characterized.
                        log -> {
							log.motor("turret-rotator")
									.voltage(
											appliedVoltage.mut_replace(
												turretRotator.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
									.angularPosition(distance.mut_replace(turretRotator.getEncoder().getPosition(), Rotations))
									.angularVelocity(
										velocity.mut_replace(turretRotator.getEncoder().getVelocity(), RotationsPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem.
                        FuelAimingSubsystem.this));

        /**
         * Returns a command that will execute a quasistatic test in the given
         * direction.
         *
         * @param direction The direction (forward or reverse) to run the test in
         */
        public Command quasistatic(SysIdRoutine.Direction direction) {
            return sysIdRoutine.quasistatic(direction);
        }

        /**
         * Returns a command that will execute a dynamic test in the given direction.
         *
         * @param direction The direction (forward or reverse) to run the test in
         */
        public Command dynamic(SysIdRoutine.Direction direction) {
            return sysIdRoutine.dynamic(direction);
        }

        /**
         * Adds system identification commands to the dashboard. Only needs to be called once.
         */
        public void configureSendables() {
            var name = FuelAimingSubsystem.this.getName();
            SmartDashboard.putData("SysId/"+name+"/Turret/Quasistatic Forward", quasistatic(SysIdRoutine.Direction.kForward));
            SmartDashboard.putData("SysId/"+name+"/Turret/Quasistatic Reverse", quasistatic(SysIdRoutine.Direction.kReverse));
            SmartDashboard.putData("SysId/"+name+"/Turret/Dynamic Forward", dynamic(SysIdRoutine.Direction.kForward));
            SmartDashboard.putData("SysId/"+name+"/Turret/Dynamic Reverse", dynamic(SysIdRoutine.Direction.kReverse));
        }
    }

	public class SysIdHood {
        private final VelocityUnit<VoltageUnit> voltsPerSeconds = Volts.per(Seconds);
		private final Velocity<VoltageUnit> rampRate = voltsPerSeconds.of(1.5);

		private final MutVoltage appliedVoltage = Volts.mutable(0);
        private final MutAngle distance = Rotations.mutable(0);
        private final MutAngularVelocity velocity = RotationsPerSecond.mutable(0);
        private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(rampRate, Volts.of(10.0), Seconds.of(10.0)),
                new SysIdRoutine.Mechanism(
                        // Drive motor controllers
                        voltage -> {
                            hoodRotator.setVoltage(voltage.baseUnitMagnitude());
						},
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being characterized.
                        log -> {
							log.motor("hood-rotator")
									.voltage(
											appliedVoltage.mut_replace(
												hoodRotator.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
									.angularPosition(distance.mut_replace(hoodRotator.getEncoder().getPosition(), Rotations))
									.angularVelocity(
										velocity.mut_replace(hoodRotator.getEncoder().getVelocity(), RotationsPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem.
                        FuelAimingSubsystem.this));

        /**
         * Returns a command that will execute a quasistatic test in the given
         * direction.
         *
         * @param direction The direction (forward or reverse) to run the test in
         */
        public Command quasistatic(SysIdRoutine.Direction direction) {
            return sysIdRoutine.quasistatic(direction);
        }

        /**
         * Returns a command that will execute a dynamic test in the given direction.
         *
         * @param direction The direction (forward or reverse) to run the test in
         */
        public Command dynamic(SysIdRoutine.Direction direction) {
            return sysIdRoutine.dynamic(direction);
        }

        /**
         * Adds system identification commands to the dashboard. Only needs to be called once.
         */
        public void configureSendables() {
            var name = FuelAimingSubsystem.this.getName();
            SmartDashboard.putData("SysId/"+name+"/Hood/Quasistatic Forward", quasistatic(SysIdRoutine.Direction.kForward));
            SmartDashboard.putData("SysId/"+name+"/Hood/Quasistatic Reverse", quasistatic(SysIdRoutine.Direction.kReverse));
            SmartDashboard.putData("SysId/"+name+"/Hood/Dynamic Forward", dynamic(SysIdRoutine.Direction.kForward));
            SmartDashboard.putData("SysId/"+name+"/Hood/Dynamic Reverse", dynamic(SysIdRoutine.Direction.kReverse));
        }
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
		builder.addBooleanProperty("Turret Setpoint Within Range", () -> turretSetpointWithinRange, null);
		builder.addBooleanProperty("Turret At Setpoint", () -> hoodController.isAtSetpoint(), null);
		builder.addBooleanProperty("Limit Switch", () -> turretAimingLimitSwitch.get(), null);

		builder.addDoubleProperty("Hood Encoder", () -> hoodRotator.getEncoder().getPosition(), null);
		builder.addDoubleProperty("Hood Power", () -> hoodRotator.get(), null);
		builder.addDoubleProperty("Hood Voltage", () -> hoodRotator.getAppliedOutput(), null);
		builder.addDoubleProperty("Hood Setpoint", () -> hoodController.getSetpoint(), null);
		builder.addDoubleProperty("Hood Velocity", () -> hoodRotator.getEncoder().getVelocity(), null);
		builder.addBooleanProperty("Hood Setpoint Within Range", () -> hoodSetpointWithinRange, null);
		builder.addBooleanProperty("Hood At Setpoint", () -> hoodController.isAtSetpoint(), null);

		builder.addBooleanProperty("In Alliance Zone", () -> inAllianceZone, null);
		builder.addDoubleProperty("Distance from Hub", () -> distanceToHub, null);
		builder.addDoubleProperty("Theta to Hub", () -> thetaToHub, null);
		builder.addDoubleProperty("Degrees Relative to Robot", () -> degreeToHubRelativeToRobot, null);
		
		builder.addDoubleProperty("Set Turret Position", () -> turretController.getSetpoint(), (position) -> hoodController.setSetpoint(position, ControlType.kMAXMotionPositionControl));
		builder.addDoubleProperty("Set Hood Position", () -> hoodController.getSetpoint(), (position) -> hoodController.setSetpoint(position, ControlType.kMAXMotionPositionControl));
		
		sysIdTurret.ifPresent(sysid -> sysid.configureSendables());
		sysIdHood.ifPresent(sysid -> sysid.configureSendables());

		//Testing
		SmartDashboard.putData("Manual Turret CW", manualTurretControlCW());
		SmartDashboard.putData("Manual Turret CCW", manualTurretControlCCW());

		SmartDashboard.putData("Manual Hood Up", manualHoodControlUp());
		SmartDashboard.putData("Manual Hood Down", manualHoodControlDown());
	}
}
//Connor's Son