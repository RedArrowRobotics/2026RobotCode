package frc.robot;

import java.io.IOException;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveOrientation;

public class RobotContainer {
    private final DriveSubsystem swerveDriveTrain;
    private final SensorInputs sensorInputs = new SensorInputs();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() throws IOException, Exception {
        swerveDriveTrain = new DriveSubsystem();

        swerveDriveTrain.setDefaultCommand(swerveDriveTrain.teleopDrive(DriveOrientation.FIELD_CENTRIC));
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    public void putDashboardData() {
        SmartDashboard.putData(sensorInputs.navxAhrs);
    }

     public Optional<Command> getAutonomousCommand() {
        // Fetch the selected autonomous command from the dashoard and put it in an
        // Optional
        return Optional.ofNullable(autoChooser.getSelected());
    }
    
    public void resetGyro() {
        swerveDriveTrain.resetGyro();
    }
}

