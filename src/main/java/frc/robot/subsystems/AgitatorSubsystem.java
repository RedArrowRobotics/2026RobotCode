package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AgitatorConstants;

public class AgitatorSubsystem extends SubsystemBase {
    private final SparkMax belt1 = new SparkMax(AgitatorConstants.BELT_MOTOR_1_ID, MotorType.kBrushless);
    private final SparkMax belt2 = new SparkMax(AgitatorConstants.BELT_MOTOR_2_ID, MotorType.kBrushless);
    private final SparkMax kicker = new SparkMax(AgitatorConstants.KICKER_MOTOR_ID, MotorType.kBrushless);


    public Command startAgitating() {
        return runOnce(() -> {
            belt1.set(AgitatorConstants.BELT_SPEED);
            belt2.set(AgitatorConstants.BELT_SPEED * -1);
            kicker.set(AgitatorConstants.KICK_SPEED);
        });
    }

    public Command stopAgitating() {
        return runOnce(() -> {
            belt1.set(0.0);
            belt2.set(0.0);
            kicker.set(0.0);
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        //Testing
        SmartDashboard.putData("Agitate", startAgitating());
        SmartDashboard.putData("Stop Agitating", stopAgitating());
    }
}
