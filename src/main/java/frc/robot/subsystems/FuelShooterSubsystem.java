package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
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
import frc.robot.Constants.FuelShooterConstants;

public class FuelShooterSubsystem extends SubsystemBase {
	private final SparkFlex shooterMotor1 = new SparkFlex(DeviceConstants.FUEL_SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
	private final SparkFlex shooterMotor2 = new SparkFlex(DeviceConstants.FUEL_SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
	private final SparkFlexConfig motor1Config = new SparkFlexConfig();
	private final SparkFlexConfig motor2Config = new SparkFlexConfig();
	private final SparkClosedLoopController shooterController = shooterMotor1.getClosedLoopController();

	private final Translation2d hubPosition = switch(DriverStation.getAlliance().orElse(Alliance.Red)) {
        case Blue -> FieldPoses.BLUE_HUB;
        case Red -> FieldPoses.RED_HUB;
    };
	
    public final Optional<SysId> sysId;
	
	public FuelShooterSubsystem () {
		if (Constants.DEBUG_ENABLED) {
            sysId = Optional.of(new SysId());
		} else {
            sysId = Optional.empty();
		}

		motor2Config.follow(DeviceConstants.FUEL_SHOOTER_MOTOR_1_ID, true);

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

		shooterMotor1.configure(motor1Config, ResetMode.kResetSafeParameters,
		PersistMode.kPersistParameters);
		shooterMotor2.configure(motor2Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
	}

	public Command shootFuelVarSpeed(Supplier<Pose2d> robotPose) {
		return run(() -> {
			//Do math to figure out optimal motor speed as a function of distance
			//Min Distance: 30 in -> 1.359m     Max Distance: 241.7 in -> 6.139m
			//39.3701 converts from inches to meters
			double distance = hubPosition.getDistance(robotPose.get().getTranslation());
			double speedParabolic = (.0544 * Math.pow(distance * 39.3701, 2)) - (2.33 * distance * 39.3701) + 2605.55;			
			shooterController.setSetpoint(speedParabolic, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
		});
	}

	public Command shootFuel() {
		return runOnce(() -> {
			shooterController.setSetpoint(FuelShooterConstants.SHOOTER_RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
		});
	}

	public Command shooterDeactivate() {
		return runOnce(() -> {
			shooterController.setSetpoint(FuelShooterConstants.STOPPED_SPEED, ControlType.kMAXMotionVelocityControl);
		});
	}

	public class SysId {
        private final MutVoltage appliedVoltage = Volts.mutable(0);
        private final MutAngle distance = Rotations.mutable(0);
        private final MutAngularVelocity velocity = RotationsPerSecond.mutable(0);
        private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Drive motor controllers
                        voltage -> {
                            shooterMotor1.setVoltage(voltage.baseUnitMagnitude());
						},
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being characterized.
                        log -> {
							log.motor("shooter-1")
									.voltage(
											appliedVoltage.mut_replace(
												shooterMotor1.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
									.angularPosition(distance.mut_replace(shooterMotor1.getEncoder().getPosition(), Rotations))
									.angularVelocity(
										velocity.mut_replace(shooterMotor1.getEncoder().getVelocity(), RotationsPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem.
                        FuelShooterSubsystem.this));

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
            var name = FuelShooterSubsystem.this.getName();
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
		builder.addDoubleProperty("Shooter Power", () -> shooterMotor1.get(), null);
		builder.addDoubleProperty("Shooter Voltage", () -> shooterMotor1.getAppliedOutput(), null);
		builder.addDoubleProperty("Shooter Setpoint", () -> shooterController.getSetpoint(), null);
		builder.addDoubleProperty("Shooter Position", () -> shooterMotor1.getEncoder().getPosition(), null);
		builder.addDoubleProperty("Shooter Velocity", () -> shooterMotor1.getEncoder().getVelocity(), null);
		builder.addBooleanProperty("Shooter At Setpoint", () -> shooterController.isAtSetpoint(), null);
		builder.addDoubleProperty("Set Shooter Setpoint", () -> shooterController.getMAXMotionSetpointVelocity(), (speed) -> shooterController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl));

        sysId.ifPresent(sysid -> sysid.configureSendables());

		//Testing
		SmartDashboard.putData("Shoot Fuel", shootFuel());
		SmartDashboard.putData("Shooter Deactivate", shooterDeactivate());
	}
}
