package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FuelShooterConstants;

public class FuelShooterSubsystem extends SubsystemBase {
	private final SparkFlex shooterMotor = new SparkFlex(DeviceConstants.FUEL_SHOOTER_MOTOR_ID, MotorType.kBrushless);
	
	public Command shootFuel() {
		return startEnd(() -> {
			shooterMotor.set(FuelShooterConstants.SHOOTER_POWER);
		}, () -> {
			shooterMotor.set(0.0);
		});
	}

	public Command shooterDeactivate() {
		return runOnce(() -> {
			
		});
	}
}
