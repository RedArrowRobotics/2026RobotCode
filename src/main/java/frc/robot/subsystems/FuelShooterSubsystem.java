package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FuelShooterConstants;

public class FuelShooterSubsystem extends SubsystemBase {
	private final SparkFlex shooterMotor1 = new SparkFlex(DeviceConstants.FUEL_SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
	private final SparkFlex shooterMotor2 = new SparkFlex(DeviceConstants.FUEL_SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
	
	public FuelShooterSubsystem () {
		SparkFlexConfig motor1Config = new SparkFlexConfig();
		SparkFlexConfig motor2Config = new SparkFlexConfig();

		motor2Config.follow(DeviceConstants.FUEL_SHOOTER_MOTOR_1_ID, true);

		shooterMotor1.configure(motor1Config, ResetMode.kResetSafeParameters,
		PersistMode.kPersistParameters);

		shooterMotor2.configure(motor2Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
	}

	public Command shootFuel() {
		return startEnd(() -> {
			shooterMotor1.set(FuelShooterConstants.SHOOTER_POWER);
		}, () -> {
			shooterMotor1.set(0.0);
		});
	}

	public Command shooterDeactivate() {
		return runOnce(() -> {
			
		});
	}
}
