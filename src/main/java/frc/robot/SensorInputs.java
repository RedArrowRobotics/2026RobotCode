package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import com.studica.frc.AHRS;

public class SensorInputs {
    //Sensor Definitions
      private final AHRS navxAhrs = new AHRS(AHRS.NavXComType.kMXP_SPI);

    //Variable Definitions
      public float currentPitchDegrees = (float) 0.0;
      public float currentYawDegrees = (float) 0.0;
      public float currentRollDegrees = (float) 0.0;
      public Rotation2d drivetrainRotation = Rotation2d.fromDegrees(0.0);

       public void readSensors() {
        SmartDashboard.putData(navxAhrs);
   }

   public final float convertTo360(float input) {
        return (input + 360) % 360;
    }
}
