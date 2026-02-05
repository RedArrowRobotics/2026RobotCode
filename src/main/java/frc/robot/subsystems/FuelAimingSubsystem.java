package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlInputs;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FeedforwardConstants;

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

	public Command manualControl() {
		return runEnd(() -> {
			
		},() -> {
			
		});
	}

	public Command automaticAimTowardsHub() {
		return runEnd(() -> {

		},() -> {
			
		});
	}
}
