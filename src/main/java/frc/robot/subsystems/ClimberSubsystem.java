package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

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
    private final SparkMaxConfig climberConfig = new SparkMaxConfig();
    private final SparkClosedLoopController climberController = climberMotor.getClosedLoopController();
    private final DigitalInput climberEncoder = new DigitalInput(DeviceConstants.CLIMBER_ENCODER_CHANNEL);
    public ClimberState climberState = ClimberState.HOME;

    public ClimberSubsystem() {
        climberConfig.closedLoop
        .p(ClimberConstants.CLIMBER_kP)
        .i(ClimberConstants.CLIMBER_kI)
        .d(ClimberConstants.CLIMBER_kD);
        climberConfig.closedLoop.feedForward
        .kV(ClimberConstants.CLIMBER_kV)
        .kS(ClimberConstants.CLIMBER_kS)
        .kA(ClimberConstants.CLIMBER_kA)
        .kG(ClimberConstants.CLIMBER_kG);
        climberConfig.closedLoop.maxMotion
        .cruiseVelocity(ClimberConstants.CLIMBER_MAX_VELOCITY)
        .maxAcceleration(ClimberConstants.CLIMBER_MAX_ACCELERATION)
        .allowedProfileError(ClimberConstants.CLIMBER_MAX_ERROR);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum ClimberState {
        HOME,
        MOVING,
        EXTENDED;
    }

    public Command climberUpManual() {
        return runOnce(() -> {
            climberMotor.set(ClimberConstants.CLIMBER_POWER);
            climberState = ClimberState.MOVING;
        });
    }

    public Command climberDownManual() {
        return runOnce(() -> {
            climberMotor.set(ClimberConstants.CLIMBER_POWER * -1);
            climberState = ClimberState.MOVING;
        });
    }

    public Command climberStop() {
        return runOnce(() -> {
            climberMotor.set(0.00000000);
            climberState = ClimberState.HOME;
        });
    }

    public Command climberUpPID() {
        return startEnd(() -> {
            climberController.setSetpoint(ClimberConstants.CLIMBER_UP_POSITION, ControlType.kMAXMotionPositionControl);
            climberState = ClimberState.MOVING;
        }, () -> {
            climberState = ClimberState.EXTENDED;
        });
    }

    public Command climberDownPID() {
         return startEnd(() -> {
            climberController.setSetpoint(ClimberConstants.CLIMBER_DOWN_POSITION, ControlType.kMAXMotionPositionControl);
            climberState = ClimberState.MOVING;
        }, () -> {
            climberState = ClimberState.HOME;
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        //Telemetry
        builder.addStringProperty("Climber State", () -> climberState.toString(), null);

        //Testing
        SmartDashboard.putData("Climber Up Manual", climberUpManual());
        SmartDashboard.putData("Climber Down Manual", climberDownManual());
        SmartDashboard.putData("Climber Stop", climberStop());
        SmartDashboard.putData("Climb Up PID", climberUpPID());
        SmartDashboard.putData("Climb Down PID", climberDownPID());
    }
}
