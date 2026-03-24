package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DeviceConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(DeviceConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    private DigitalInput climberEncoder = new DigitalInput(DeviceConstants.CLIMBER_ENCODER_CHANNEL);

    public Command climberUpManual() {
        return runOnce(() -> {
            climberMotor.set(ClimberConstants.CLIMBER_POWER);
        });
    }

    public Command climberStop() {
        return runOnce(() -> {
            climberMotor.set(0.00000000);
        });
    }

    public Command climberDownManual() {
        return runOnce(() -> {
            climberMotor.set(-1 * ClimberConstants.CLIMBER_POWER);
        });
    }

    public Command climberUpPID() {
        return runOnce(() -> {

        });
    }

    public Command climberDownPID() {
        return runOnce(() -> {

        });
    }

    public Command climberUpCommand(Supplier<Boolean> manualControlled) {
        return null;
    }

    public Command climberDownCommand(Supplier<Boolean> manualControlled) {
        return null;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        //Testing
        SmartDashboard.putData("Climb Up", climberUpManual());
        SmartDashboard.putData("Climb Down", climberDownManual());
    }
}
