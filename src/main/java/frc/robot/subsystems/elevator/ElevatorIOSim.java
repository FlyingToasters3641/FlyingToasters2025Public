package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;

public class ElevatorIOSim implements ElevatorIO{
    

    private final ElevatorSim EL_sim = new ElevatorSim(
        DCMotor.getKrakenX60(4),
        4, 
        Pounds.of(9.8).in(Kilograms), //TODO: Ask for Intake Mass 
        Inches.of(2).in(Meters), //TODO: Ask for Spool Radius
        Inches.of(22.4).in(Meters), 
        Inches.of(77).in(Meters), 
        true, 
        Inches.of(0).in(Meters)
    );
    

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.EL_position = EL_sim.getPositionMeters();
    }


    //TODO: Figure out how to convert voltage into movement of position. TRY PHOENIX SIM!
    @Override
    public void setELPosition(double position){
        double currentPosition = EL_sim.getPositionMeters();
    }

}
