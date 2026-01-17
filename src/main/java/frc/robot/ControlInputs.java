package frc.robot;

import frc.robot.subsystems.DriveSubsystem.DrivePower;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class ControlInputs {
      // Joysticks
    private final Joystick driveController = new Joystick(0);
    private final GenericHID componentsBoard = new GenericHID(1);
    public DrivePower getdriveController() {
        // Multipliers for the drive stick axes
        final double driveControllerLinearMultiplier = 0.8;
        final double driveControllerRotationMultiplier = 0.5;

         // Derive joystick values. Inputs are squared to make precice control at low speeds easier
        var x = ( driveController.getX() * Math.abs( driveController.getX()) ) * driveControllerLinearMultiplier;
        var y = ( driveController.getY() * Math.abs( driveController.getY()) ) * driveControllerLinearMultiplier;
        var rotation = ( driveController.getZ() * Math.abs( driveController.getZ()) ) * driveControllerRotationMultiplier;
        // Compose the seperate components into a state record
        return new DrivePower(-x, y, rotation);
    }

     public class Triggers {
        public Triggers() {
            componentsBoard.setOutput(1, false);
        }

      public final Trigger slowSpeed = new Trigger(() -> driveController.getRawButton(1) );
    }
}
