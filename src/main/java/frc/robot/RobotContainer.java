package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.util.Optional;
import java.util.ResourceBundle.Control;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveOrientation;
import frc.robot.subsystems.FuelAimingSubsystem;
import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.subsystems.FuelShooterSubsystem;
import frc.robot.subsystems.HopperSubsytem;

public class RobotContainer {
    private final DriveSubsystem swerveDriveTrain;
    private final FuelIntakeSubsystem fuelIntake = new FuelIntakeSubsystem();
    private final FuelAimingSubsystem fuelAiming = new FuelAimingSubsystem();
    private final FuelShooterSubsystem fuelShooter = new FuelShooterSubsystem();
    private final AgitatorSubsystem agitator = new AgitatorSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final HopperSubsytem hopper = new HopperSubsytem();
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() throws IOException, Exception {
        swerveDriveTrain = new DriveSubsystem();

        fuelShooter.setDefaultCommand(fuelShooter.shooterDeactivate());
        agitator.setDefaultCommand(agitator.stopAgitating());
        fuelIntake.setDefaultCommand(fuelIntake.intakeStop());
        fuelAiming.setDefaultCommand(fuelAiming.automaticAimRoutine(() -> swerveDriveTrain.getPose()));
        autoChooser = AutoBuilder.buildAutoChooser();

        configureBindings();
        configureSendables();
    }

    private void configureBindings() {
        ControlInputs.componentsBoard.button(InputConstants.SHOOT_FUEL).whileTrue(fuelShooter.shootFuelVarSpeed(() -> swerveDriveTrain.getPose()).alongWith(agitator.startAgitating()));
        ControlInputs.componentsBoard.button(InputConstants.INTAKE_OUT).whileFalse(fuelIntake.intakeFuelOut());
        ControlInputs.componentsBoard.button(InputConstants.INTAKE_IN).whileFalse(fuelIntake.intakeFuelIn());
        ControlInputs.componentsBoard.button(InputConstants.EXTEND_HOPPER).onTrue(hopper.extendHopper());
        ControlInputs.componentsBoard.button(InputConstants.EXTEND_HOPPER).onFalse(hopper.retractHopper());
        ControlInputs.componentsBoard.button(InputConstants.CLIMBER_DOWN).whileTrue(climber.climberDownCommand(() -> ControlInputs.componentsBoard.button(InputConstants.MANUAL_SWITCH).getAsBoolean()));
        ControlInputs.componentsBoard.button(InputConstants.CLIMBER_UP).whileTrue(climber.climberUpCommand(() -> ControlInputs.componentsBoard.button(InputConstants.MANUAL_SWITCH).getAsBoolean()));
        ControlInputs.componentsBoard.button(InputConstants.MANUAL_HOOD_UP).whileTrue(fuelAiming.manualHoodControlUp(() -> ControlInputs.componentsBoard.button(InputConstants.MANUAL_SWITCH).getAsBoolean()));
        ControlInputs.componentsBoard.button(InputConstants.MANUAL_HOOD_DOWN).whileFalse(fuelAiming.manualHoodControlDown(() -> ControlInputs.componentsBoard.button(InputConstants.MANUAL_SWITCH).getAsBoolean()));
        ControlInputs.componentsBoard.button(InputConstants.MANUAL_TURRET_CCW).whileTrue(fuelAiming.manualTurretControlCCW(() -> ControlInputs.componentsBoard.button(InputConstants.MANUAL_SWITCH).getAsBoolean()));
        ControlInputs.componentsBoard.button(InputConstants.MANUAL_TURRET_CW).whileTrue(fuelAiming.manualTurretControlCW(() -> ControlInputs.componentsBoard.button(InputConstants.MANUAL_SWITCH).getAsBoolean()));
        ControlInputs.componentsBoard.button(InputConstants.DRIVE_ORIENTATION_SWITCH).whileFalse(swerveDriveTrain.teleopDrive(DriveOrientation.FIELD_CENTRIC));
        ControlInputs.componentsBoard.button(InputConstants.DRIVE_ORIENTATION_SWITCH).whileTrue(swerveDriveTrain.teleopDrive(DriveOrientation.ROBOT_CENTRIC));

        NamedCommands.registerCommand("Zero Turret", fuelAiming.zeroTurret());
        NamedCommands.registerCommand("Aim Routine", fuelAiming.automaticAimRoutine(() -> swerveDriveTrain.getPose()).until(() -> fuelAiming.turretAtSetpoint()));
        NamedCommands.registerCommand("Shoot Fuel Var Speed", fuelShooter.shootFuelVarSpeed(() -> swerveDriveTrain.getPose()).withTimeout(Seconds.of(5.0)));
        NamedCommands.registerCommand("Agitate Fuel", agitator.startAgitating().withTimeout(Seconds.of(5.0)));
        NamedCommands.registerCommand("Climber Up", climber.climberUpPID());
        NamedCommands.registerCommand("Climber Down", climber.climberDownPID());
        NamedCommands.registerCommand("Hopper In", hopper.retractHopper());
        NamedCommands.registerCommand("Hopper Out", hopper.extendHopper());
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
