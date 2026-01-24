package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelIntakeSubsystem extends SubsystemBase {

	public Command intakeIn() {
		return runOnce(() -> {

		});
	}

	public Command intakeOut() {
		return runOnce(() -> {
			
		});
	}

	public Command intakeStop() {
		return runOnce(() -> {
			
		});
	}

	public Command extendIntake() {
		return runOnce(() -> {
			
		});
	}

	public Command retractIntake() {
		return runOnce(() -> {
			
		});
	}

	public boolean genericSensor() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

}
