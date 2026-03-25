package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.IntakeConstants;

public class FuelIntakeSubsystem extends SubsystemBase {
	private final SparkFlex intakeBar = new SparkFlex(DeviceConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

	public Command intakeFuelIn() {
		return runOnce(() -> {
			intakeBar.set(IntakeConstants.INTAKE_SPEED);
		});
	}

	public Command intakeFuelOut() {
		return runOnce(() -> {
			intakeBar.set(IntakeConstants.INTAKE_SPEED * -1);
		});
	}

	public Command intakeStop() {
		return runOnce(() -> {
			intakeBar.set(0.0);
		});
	}

	

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		//Testing
		SmartDashboard.putData("Intake Fuel In", intakeFuelIn());
		SmartDashboard.putData("Intake Fuel Out", intakeFuelOut());
	}
}
