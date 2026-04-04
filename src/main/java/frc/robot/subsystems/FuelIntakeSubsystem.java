package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.IntakeConstants;

public class FuelIntakeSubsystem extends SubsystemBase {
	private final SparkMax intakeBar = new SparkMax(DeviceConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

	public Command intakeFuelIn() {
		return startEnd(() -> {
			intakeBar.set(IntakeConstants.INTAKE_SPEED);
		}, () -> {
			intakeBar.set(0.0);
		});
	}

	public Command intakeFuelOut() {
		return startEnd(() -> {
			intakeBar.set(IntakeConstants.INTAKE_SPEED * -1);
		}, () -> {
			intakeBar.set(0.0);
		});
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		//Telemetry
		builder.addDoubleProperty("Set Intake Power", () -> intakeBar.get(), (power) -> intakeBar.set(power));

		//Testing
		SmartDashboard.putData("Intake Fuel In", intakeFuelIn());
		SmartDashboard.putData("Intake Fuel Out", intakeFuelOut());
	}
}
