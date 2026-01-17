package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.Optional;
import java.util.Set;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveOrientation;

public class RobotContainer {
  
    private final ControlInputs controlInputs = new ControlInputs();
    private final ControlInputs.Triggers controlTriggers = controlInputs.new Triggers();
    private final DriveSubsystem swerveDriveTrain;
    private final SensorInputs sensorInputs = new SensorInputs();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() throws IOException, Exception {
        swerveDriveTrain = new DriveSubsystem();

        swerveDriveTrain.setDefaultCommand( 
                swerveDriveTrain.teleopDrive(
                () -> {
                    var power = controlInputs.getdriveController().toSwerve();
                    if (controlTriggers.slowSpeed.getAsBoolean()) {
                        power = power.times(0.5);
                    }
                    return power;
                },
                DriveOrientation.FIELD_CENTRIC)
                );
      
         
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    public void robotPeriodic() {
        sensorInputs.readSensors();
    }

    public void teleopPeriodic() {
    
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

