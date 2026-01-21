package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public final class Alerts {
    public static final Alert robotInit = new Alert("An unrecoverable error occured while setting up the robot. Check logs for more info.", AlertType.kError);
    
}
