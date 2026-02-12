package frc.robot;

import java.io.IOException;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveOrientation;
import frc.robot.subsystems.FuelAimingSubsystem;
import frc.robot.subsystems.FuelIntakeSubsystem;
//import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.subsystems.FuelShooterSubsystem;

public class RobotContainer {
    private final DriveSubsystem swerveDriveTrain;
    private final FuelIntakeSubsystem fuelIntake = new FuelIntakeSubsystem();
    private final FuelAimingSubsystem fuelAiming = new FuelAimingSubsystem();
    private final FuelShooterSubsystem fuelShooter = new FuelShooterSubsystem();
    private final AgitatorSubsystem agitator = new AgitatorSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() throws IOException, Exception {
        swerveDriveTrain = new DriveSubsystem();

        swerveDriveTrain.setDefaultCommand(swerveDriveTrain.teleopDrive(DriveOrientation.FIELD_CENTRIC));
        fuelShooter.setDefaultCommand(fuelShooter.shooterDeactivate());
        autoChooser = AutoBuilder.buildAutoChooser();

        configureBindings();
    }

    private void configureBindings() {
        ControlInputs.componentsBoard.button(1).onTrue(fuelShooter.shootFuel());
        ControlInputs.componentsBoard.button(2).onTrue(fuelIntake.intakeIn());
        ControlInputs.componentsBoard.button(3).onTrue(fuelIntake.intakeOut());
        ControlInputs.componentsBoard.axisGreaterThan(1, 0.5).onTrue(fuelAiming.manualTurretControlCW());
        ControlInputs.componentsBoard.axisLessThan(1, -0.5).onTrue(fuelAiming.manualTurretControlCCW());
    }

    public void putDashboardData() {
        SmartDashboard.putData(SensorInputs.navxAhrs);
        SmartDashboard.putData(fuelShooter);
        SmartDashboard.putData(fuelAiming);
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

