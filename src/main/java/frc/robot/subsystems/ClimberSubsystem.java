package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(DeviceConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

    public Command climberAscend() {
        return runOnce(() -> {
            climberMotor.set(ClimberConstants.CLIMBER_POWER);
        });
    }

    public Command climberStop() {
        return runOnce(() -> {
            climberMotor.set(0.00000000);
        });
    }

    public Command climberDescend() {
        return runOnce(() -> {
            climberMotor.set(-1 * ClimberConstants.CLIMBER_POWER);
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        //Testing
        SmartDashboard.putData("Climb Up", climberAscend());
        SmartDashboard.putData("Climb Down", climberDescend());
    }
}
