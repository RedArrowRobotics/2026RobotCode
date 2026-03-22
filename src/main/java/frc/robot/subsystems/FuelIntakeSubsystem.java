package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FeedforwardConstants;
import frc.robot.Constants.IntakeConstants;

public class FuelIntakeSubsystem extends SubsystemBase {
	private final SparkFlex intakeBar = new SparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

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
