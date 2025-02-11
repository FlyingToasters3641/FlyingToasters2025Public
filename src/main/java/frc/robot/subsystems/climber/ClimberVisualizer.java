package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;

public class ClimberVisualizer {
    private final String key;

    public ClimberVisualizer(String key) {
        this.key = key;
    }

    public void update(Angle rotation) {
        Logger.recordOutput("Climber/Mechanism3d/" + key, rotation);
    }
  }