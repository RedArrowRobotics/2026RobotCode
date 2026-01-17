// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private Optional<Command> autonomousCommand = Optional.empty();

  private RobotContainer robotContainer;
  private PowerDistribution powerDistribution;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    try {
      robotContainer = new RobotContainer();
    } catch (Exception e) {
      System.out.println("Exception caught in Robot()");
      e.printStackTrace();
    }
    powerDistribution = new PowerDistribution(20,ModuleType.kRev);
  }

  @Override
  public void robotPeriodic() {
    robotContainer.robotPeriodic();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    {
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
      SmartDashboard.putData(powerDistribution);
    }
  }

  @Override
  public void autonomousInit() {
    robotContainer.resetGyro();
    // If we have an autonomous command selected, schedule it to be ran.
    autonomousCommand = robotContainer.getAutonomousCommand();
    autonomousCommand.ifPresent(Command::schedule);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
     // If we were running an autonomous command, cancel it when teleop starts.
    autonomousCommand.ifPresent(Command::cancel);
  }

  @Override
  public void teleopPeriodic() {
     robotContainer.teleopPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
     // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
