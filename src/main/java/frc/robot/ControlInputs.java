package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class ControlInputs {
    // Joysticks
    public static final CommandJoystick driveController = new CommandJoystick(OperatorConstants.DRIVE_JOYSTICK_PORT);
    public static final CommandGenericHID componentsBoard = new CommandGenericHID(OperatorConstants.COMPONENTS_BOARD_PORT);

    private static final Alert driveControllerAlert = new Alert("The drive joystick is not connected to the driver's station.", AlertType.kWarning);
    private static final Alert componentsBoardAlert = new Alert("The components control board is not connected to the driver's station.", AlertType.kWarning);
    
    public static Twist2d getDrivePower() {
        // Multipliers for the drive stick axes
        final double driveControllerLinearMultiplier = 0.8;
        final double driveControllerRotationMultiplier = 0.5;

        // Derive joystick values. Inputs are squared to make precice control at low speeds easier
        var x = (driveController.getX() * Math.abs(driveController.getX()))
                * driveControllerLinearMultiplier;
        var y = (driveController.getY() * Math.abs(driveController.getY()))
                * driveControllerLinearMultiplier;
        var rotation = (driveController.getZ() * Math.abs(driveController.getZ()))
                * driveControllerRotationMultiplier;
        // Compose the seperate components into a state record
        return new Twist2d(x, y, -rotation);
    }

    public static void updateAlerts() {
        driveControllerAlert.set(!driveController.isConnected());
        componentsBoardAlert.set(!componentsBoard.isConnected());
    }

     // For later
    public final void setRumble(double value) {
        driveController.setRumble(RumbleType.kBothRumble, value);
    }
}
