package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
        configureSendables();
    }

    private void configureBindings() {
        ControlInputs.componentsBoard.axisGreaterThan(1, 0.5).onTrue(fuelAiming.manualTurretControlCW());
        ControlInputs.componentsBoard.axisLessThan(1, -0.5).onTrue(fuelAiming.manualTurretControlCCW());
        //ControlInputs.componentsBoard.button(6).onTrue(fuelIntake.extendIntake());
        //ControlInputs.componentsBoard.button(7).onTrue(fuelIntake.retractIntake());
        ControlInputs.componentsBoard.button(0).and(() -> fuelAiming.setpointWithinRange).whileTrue(fuelShooter.shootFuel());
        //ControlInputs.componentsBoard.button(1).onTrue(fuelIntake.intakeFuelIn());
        //ControlInputs.componentsBoard.button(2).onTrue(fuelIntake.intakeFuelOut());
        //ControlInputs.componentsBoard.button(3).whileTrue(climber.climberAscend());
        //ControlInputs.componentsBoard.button(4).whileTrue(climber.climberDescend());
        //ControlInputs.componentsBoard.button(5).whileTrue(agitator.startAgitating());

        //ControlInputs.componentsBoard.button(6).whileTrue(fuelAiming.automaticAimRoutine(() -> swerveDriveTrain.getPose()));
        ControlInputs.componentsBoard.button(7).whileTrue(fuelShooter.shootFuelVarSpeed(() -> swerveDriveTrain.getPose()));

        NamedCommands.registerCommand("Zero Turret", fuelAiming.zeroTurret());
        NamedCommands.registerCommand("Aim Routine", fuelAiming.automaticAimRoutine(() -> swerveDriveTrain.getPose()).until(() -> fuelAiming.turretAtSetpoint()));
        NamedCommands.registerCommand("Shoot Fuel Var Speed", fuelShooter.shootFuelVarSpeed(() -> swerveDriveTrain.getPose()).withTimeout(Seconds.of(5.0)));
        NamedCommands.registerCommand("Agitate Fuel", agitator.startAgitating().withTimeout(Seconds.of(5.0)));
        NamedCommands.registerCommand("Climber Up", climber.climberUpPID());
        NamedCommands.registerCommand("Climber Down", climber.climberDownPID());
        NamedCommands.registerCommand("Hopper In", fuelIntake.retractIntakePIDF());
        NamedCommands.registerCommand("Hopper Out", fuelIntake.extendIntakePIDF());
        NamedCommands.registerCommand("Intake Fuel", fuelIntake.intakeFuelIn());
        NamedCommands.registerCommand("'Barf' Fuel", fuelIntake.intakeFuelOut());
        NamedCommands.registerCommand("Stop Intake", fuelIntake.intakeStop());
    }

    /**
     * Adds sendable data to the dashboard. Sendable data will automatically update
     * each iteration of the robot loop, so this function only needs to be called
     * once.
     */
    private void configureSendables() {
        SmartDashboard.putData(SensorInputs.navxAhrs);
        SmartDashboard.putData(fuelShooter);
        SmartDashboard.putData(agitator);
        SmartDashboard.putData(fuelAiming);
        SmartDashboard.putData(swerveDriveTrain);
        SmartDashboard.putData("Aim Routine", fuelAiming.automaticAimRoutine(() -> swerveDriveTrain.getPose()));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Optional<Command> getAutonomousCommand() {
         //Fetch the selected autonomous command from the dashoard and put it in an
         //Optional
         return Optional.ofNullable(autoChooser.getSelected());
    }

    public void resetGyro() {
        swerveDriveTrain.resetGyro();
    }

    public void robotPeriodic() {
        
    }
}
