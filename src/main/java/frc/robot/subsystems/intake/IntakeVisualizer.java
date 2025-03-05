package frc.robot.subsystems.intake;

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

public class IntakeVisualizer {

    private String key;
    private LoggedMechanism2d panel;
    private LoggedMechanismLigament2d intake;
    private LoggedMechanismRoot2d root;

    public IntakeVisualizer(String key, Color color) {
        this.key = key;
        this.panel = new LoggedMechanism2d(Inches.of(100).in(Meters), Inches.of(20).in(Meters),
                new Color8Bit(Color.kWhite));
        this.root = panel.getRoot("intake", Inches.of(0).in(Meters), Inches.of(10).in(Meters));
        this.intake = root.append(
                new LoggedMechanismLigament2d(
                        "Intake",
                        Inches.of(20).in(Meters),
                        0,
                        10,
                        new Color8Bit(color)));
        Logger.recordOutput("Intake/Mechanism2d/" + key, this.panel);
    }

    public void update(Angle position) {
        intake.setAngle(position.in(Radians));
        Logger.recordOutput("Intake/Mechanism2d/" + key, this.panel);
        Angle intakeAngle = position.times(IntakeConstants.INTAKE_ANGLE_MODIFIER);
        Pose3d intake3d = new Pose3d(Inches.zero(), Inches.zero(), Inches.zero(), new Rotation3d(0, 0, intakeAngle.in(Degrees)));
        Logger.recordOutput("Intake/Mechanism3d/" + key, intake3d);
    }
}
