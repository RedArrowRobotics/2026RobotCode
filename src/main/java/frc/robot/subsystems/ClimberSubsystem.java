package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DeviceConstants;
import frc.robot.Constants.FeedforwardConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(DeviceConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig climberConfig = new SparkMaxConfig();
    private final SparkClosedLoopController climberController = climberMotor.getClosedLoopController();
    private final DigitalInput climberEncoder = new DigitalInput(DeviceConstants.CLIMBER_ENCODER_CHANNEL);

    public ClimberSubsystem() {
        climberConfig.closedLoop
        .p(FeedforwardConstants.CLIMBER_kP)
        .i(FeedforwardConstants.CLIMBER_kI)
        .d(FeedforwardConstants.CLIMBER_kD);
        climberConfig.closedLoop.feedForward
        .kV(FeedforwardConstants.CLIMBER_kV)
        .kS(FeedforwardConstants.CLIMBER_kS)
        .kA(FeedforwardConstants.CLIMBER_kA)
        .kG(FeedforwardConstants.CLIMBER_kG);
        climberConfig.closedLoop.maxMotion
        .cruiseVelocity(FeedforwardConstants.CLIMBER_MAX_VELOCITY)
        .maxAcceleration(FeedforwardConstants.CLIMBER_MAX_ACCELERATION)
        .allowedProfileError(FeedforwardConstants.CLIMBER_MAX_ERROR);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climberController.setSetpoint(0.0, ControlType.kMAXMotionPositionControl);
    }

    public Command climberUpManual() {
        return runEnd(() -> {
            if(climberMotor.getEncoder().getPosition() <= ClimberConstants.CLIMBER_UP_POSITION) {
                climberMotor.set(ClimberConstants.CLIMBER_POWER);
            } else {
                climberMotor.set(0.0);
            }
        }, () -> {
            climberMotor.set(0.0);
        });
    }

    public Command climberDownManual() {
        return startEnd(() -> {
            climberMotor.set(ClimberConstants.CLIMBER_POWER * -1);
        }, () -> {
            climberMotor.set(0.0);
        });
    }

    public Command climberUpPID() {
        return runOnce(() -> {
            climberController.setSetpoint(ClimberConstants.CLIMBER_UP_POSITION, ControlType.kMAXMotionPositionControl);
        });
    }

    public Command climberDownPID() {
         return runOnce(() -> {
            climberController.setSetpoint(ClimberConstants.CLIMBER_DOWN_POSITION, ControlType.kMAXMotionPositionControl);
        });
    }

    public Command climberUpCommand(Supplier<Boolean> manualControlled, Supplier<Boolean> hopperHome) {
        Command controlType;
        if(manualControlled.get() == true) {
            controlType = climberUpManual();
        } else {
            controlType = climberUpPID();
        }
        return controlType;
    }

    public Command climberDownCommand(Supplier<Boolean> manualControlled, Supplier<Boolean> hopperHome) {
        Command controlType;
        if(hopperHome.get() == true) {
            if(manualControlled.get() == true) {
                controlType = climberDownManual();
            } else {
                controlType = climberDownPID();
            }
        } else {
            controlType = runOnce(() -> {});
        }
        return controlType;
    }

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final VelocityUnit<VoltageUnit> voltsPerSecond = Volts.per(Seconds);
    private final Velocity<VoltageUnit> rampRate = voltsPerSecond.of(0.2);
    private final Voltage dynamicVoltage = Volts.of(7.0);
    private final Time runTime = Seconds.of(30.0);

            // Creates a SysIdRoutine
            SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(rampRate, dynamicVoltage, runTime),
                new SysIdRoutine.Mechanism(voltage -> {
                        climberMotor.setVoltage(voltage.baseUnitMagnitude());
                        },
                        // Tell SysId how to record a frame of data for each motor on the mechanism being
                        // characterized.
                        log -> {
                            // Record a frame for the left motors.  Since these share an encoder, we consider
                            // the entire group to be one motor.
                            log.motor("climber")
                                .voltage(
                                    m_appliedVoltage.mut_replace(
                                        climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(m_distance.mut_replace(climberMotor.getEncoder().getPosition(), Meters))
                                .linearVelocity(
                                    m_velocity.mut_replace(climberMotor.getEncoder().getVelocity(), MetersPerSecond));
                                }, ClimberSubsystem.this)
            );

        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return routine.quasistatic(direction);
        }

        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return routine.dynamic(direction);
        }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        //Telemetry
        builder.addDoubleProperty("Climber Position", () -> climberMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Set Climber Position", () -> climberController.getSetpoint(), (pos) -> climberController.setSetpoint(pos, ControlType.kMAXMotionPositionControl));
        builder.addDoubleProperty("Set Climber Power", () -> climberMotor.get(), (speed) -> climberMotor.set(speed));

        //Testing
        SmartDashboard.putData("Climber - Dynamic Forward", sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("Climber - Dynamic Reverse", sysIdDynamic(Direction.kReverse));
        SmartDashboard.putData("Climber - Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("Climber - Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));

        SmartDashboard.putData("Climber Up Manual", climberUpManual());
        SmartDashboard.putData("Climber Down Manual", climberDownManual());
        SmartDashboard.putData("Climb Up PID", climberUpPID());
        SmartDashboard.putData("Climb Down PID", climberDownPID());
    }
}
