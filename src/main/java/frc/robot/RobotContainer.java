package frc.robot;

import java.io.IOException;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveOrientation;
import frc.robot.subsystems.FuelIntakeSubsystem;
//import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.subsystems.FuelShooterSubsystem;

public class RobotContainer {
    private final DriveSubsystem swerveDriveTrain;
    private final FuelIntakeSubsystem fuelIntake = new FuelIntakeSubsystem();
    private final FuelShooterSubsystem fuelShooter = new FuelShooterSubsystem();
    private final AgitatorSubsystem agitator = new AgitatorSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() throws IOException, Exception {
        swerveDriveTrain = new DriveSubsystem();

        swerveDriveTrain.setDefaultCommand(swerveDriveTrain.teleopDrive(DriveOrientation.FIELD_CENTRIC));
        autoChooser = AutoBuilder.buildAutoChooser();

        configureBindings();
    }

    private void configureBindings() {
        ControlInputs.componentsBoard.button(0).onTrue(fuelShooter.shootFuel());
        ControlInputs.componentsBoard.button(1).onTrue(fuelIntake.intakeIn());
        ControlInputs.componentsBoard.button(2).onTrue(fuelIntake.intakeOut());
        ControlInputs.componentsBoard.button(6).onTrue(agitator.startAgitating());
        ControlInputs.componentsBoard.button(7).onTrue(agitator.stopAgitating());

        NamedCommands.registerCommand("Shoot Fuel", fuelShooter.shootFuel());
    }

    public void putDashboardData() {
        SmartDashboard.putData(SensorInputs.navxAhrs);
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

