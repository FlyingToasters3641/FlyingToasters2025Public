package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.*;

public class ElevatorVisualizer {
    private final String key;
    private final LoggedMechanism2d panel;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d elevator;


    private LoggedMechanism2d ElevatorLogged;

    //Copied over from the training video for nowsies. ;)

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
        Logger.recordOutput("Elevator/Mechanism2d/" + key, this.panel); //TODO: Find a better way to do this so that it gets incorporated with every subsystem
        Pose3d elevator3d = new Pose3d(Inches.zero(), Inches.zero(), position, new Rotation3d()); //3d view changed to be a straight line up.
        Logger.recordOutput("Elevator/Mechanism3d/" + key, elevator3d);

        Logger.recordOutput("Elevator/ComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), elevator3d, elevator3d}); 
    }

    
  }
