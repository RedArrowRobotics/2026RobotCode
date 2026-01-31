package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlInputs;
import frc.robot.Constants.DeviceConstants;

public class FuelAimingSubsystem extends SubsystemBase {
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
