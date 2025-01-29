package frc.robot.subsystems.scorer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.*;

public class ScorerVisualizer {

    private String key;
    private LoggedMechanism2d panel;
    private LoggedMechanismLigament2d Scorer;
    private LoggedMechanismRoot2d root;

    public ScorerVisualizer(String key, Color color) {
        this.key = key;
        this.panel = new LoggedMechanism2d(Inches.of(100).in(Meters), Inches.of(19).in(Meters),
                new Color8Bit(Color.kWhite));
        this.root = panel.getRoot("Scorer", Inches.of(0).in(Meters), Inches.of(10).in(Meters));
        this.Scorer = root.append(
                new LoggedMechanismLigament2d(
                        "Scorer",
                        Inches.of(19).in(Inches),
                        0,
                        10,
                        new Color8Bit(color)));
        Logger.recordOutput("Scorer/Mechanism2d/" + key, this.panel);
    }

    public void update(Angle position) {
        Scorer.setAngle(position.in(Radians));
        Logger.recordOutput("Scorer/Mechanism2d/" + key, this.panel);
        Angle ScorerAngle = position.times(ScorerConstants.SCORER_ANGLE_MODIFIER);
        Pose3d Scorer3d = new Pose3d(Inches.zero(), Inches.zero(), Inches.zero(), new Rotation3d(0, 0, ScorerAngle.in(Radians)));
        Logger.recordOutput("Scorer/Mechanism3d/" + key, Scorer3d);
    }
}
