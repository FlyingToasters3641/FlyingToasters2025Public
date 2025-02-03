package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

public class ElevatorVisualizer {
    private final String key;
    private final LoggedMechanism2d panel;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d elevator;



    public ElevatorVisualizer(String key, Color color) {
        this.key = key;

        //Programs a straight line to represent the elevator movements.
        this.panel = new LoggedMechanism2d(Inches.of(100).in(Meters), Inches.of(100).in(Meters), new Color8Bit(Color.kWhite));
        this.root = panel.getRoot("elevator", Inches.of(7.35).in(Meters), Inches.of(10).in(Meters));
        this.elevator = root.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                Inches.of(0).in(Meters),
                90,
                10,
                new Color8Bit(color)
            )
        );



        Logger.recordOutput("Elevator/Mechanism2d/" + key, this.panel);
    }

    public void update(Distance position) {
        elevator.setLength(position.in(Meters));
        Logger.recordOutput("Elevator/Mechanism2d/" + key, this.panel);
       
    }


    
    
  }
