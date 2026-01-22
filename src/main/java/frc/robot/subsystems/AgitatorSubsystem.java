package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem {

    public Command startAgitating() {
        return runOnce(
            () -> {
                
            });

    }

    public Command stopAgitating(){
        return runOnce(
            () -> {

            });
    }

}
