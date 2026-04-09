package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FeedforwardConstants;

public class AgitatorSubsystem extends SubsystemBase {
    private final SparkMax kicker = new SparkMax(DeviceConstants.KICKER_MOTOR_ID, MotorType.kBrushless);
    public final SparkMax spinner = new SparkMax(DeviceConstants.SPINNER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig kickerConfig = new SparkMaxConfig();
    private final SparkClosedLoopController kickerController = kicker.getClosedLoopController();
    private final MakeKickerWork codeMagic = new MakeKickerWork();

    public AgitatorSubsystem() {
        kickerConfig.closedLoop
        .p(FeedforwardConstants.KICKER_kP)
        .i(FeedforwardConstants.KICKER_kI)
        .d(FeedforwardConstants.KICKER_kD);
        kickerConfig.closedLoop.feedForward
        .kV(FeedforwardConstants.KICKER_kV)
        .kS(FeedforwardConstants.KICKER_kS)
        .kA(FeedforwardConstants.KICKER_kA);
        kickerConfig.closedLoop.maxMotion
        .cruiseVelocity(FeedforwardConstants.HOOD_ROTATOR_MAX_VELOCITY)
        .maxAcceleration(FeedforwardConstants.HOOD_ROTATOR_MAX_ACCELERATION)
        .allowedProfileError(FeedforwardConstants.HOOD_ROTATOR_MAX_ERROR);

        kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command dontBreakTheKicker() {
        return run(() -> {
            codeMagic.work();
        });
    }

    public Command agitateIn() {
        return startEnd(() -> {
            kickerController.setSetpoint(AgitatorConstants.KICK_RPM, ControlType.kMAXMotionVelocityControl);
            //spinner.set(AgitatorConstants.SPIN_SPEED);
        }, () -> {
            kickerController.setSetpoint(0.0, ControlType.kMAXMotionVelocityControl);
            //spinner.set(0.0);
        });
    }

    public Command agitateOut() {
        return startEnd(() -> {
            kickerController.setSetpoint(AgitatorConstants.KICK_RPM * -1, ControlType.kMAXMotionVelocityControl);
            spinner.set(AgitatorConstants.SPIN_SPEED * -1);
        }, () -> {
            kickerController.setSetpoint(0.0, ControlType.kMAXMotionVelocityControl);
            spinner.set(0.0);
        });
    }

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_distance = Rotations.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);


	// Creates a SysIdRoutine
	SysIdRoutine kickerRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(voltage -> {
				kicker.setVoltage(voltage.baseUnitMagnitude());
				},
				// Tell SysId how to record a frame of data for each motor on the mechanism being
				// characterized.
				log -> {
					// Record a frame for the left motors.  Since these share an encoder, we consider
					// the entire group to be one motor.
					log.motor("kicker")
						.voltage(
							m_appliedVoltage.mut_replace(
								kicker.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
						.angularPosition(m_distance.mut_replace(kicker.getEncoder().getPosition(), Rotations))
						.angularVelocity(
							m_velocity.mut_replace(kicker.getEncoder().getVelocity(), RotationsPerSecond));
						}, this)
	);

	public Command sysIdQuasistaticKicker(SysIdRoutine.Direction direction) {
  		return kickerRoutine.quasistatic(direction);
	}

	public Command sysIdDynamicKicker(SysIdRoutine.Direction direction) {
  		return kickerRoutine.dynamic(direction);
	}

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        //Telemetry
        builder.addDoubleProperty("Kicker Position", () -> kicker.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Set Kicker Setpoint", () -> kickerController.getMAXMotionSetpointVelocity(), (setpoint) -> kickerController.setSetpoint(setpoint, ControlType.kMAXMotionVelocityControl));
        
        //Sys ID
        SmartDashboard.putData("Kicker - Run Forward Dynamic", sysIdDynamicKicker(Direction.kForward));
        SmartDashboard.putData("Kicker - Run Reverse Dynamic", sysIdDynamicKicker(Direction.kReverse));
        SmartDashboard.putData("Kicker - Run Forward Quasistatic", sysIdQuasistaticKicker(Direction.kForward));
        SmartDashboard.putData("Kicker - Run Reverse Quasistatic", sysIdQuasistaticKicker(Direction.kReverse));

        //Testing
        SmartDashboard.putData("Agitate In", agitateIn());
        SmartDashboard.putData("Agitate Out", agitateOut());
    }
}
