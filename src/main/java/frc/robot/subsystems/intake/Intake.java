package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private IntakeIO io;


    IntakeIOTalonFX IntakeTalonFX = new IntakeIOTalonFX();
    public static double frontTalonVoltage;
    public Intake(IntakeIO io) {
        this.io = io;
     
    }

    @Override
    public void periodic() {


    }

    public void lowerJointMotor(DoubleSupplier axis) { //This is supposed to lower the intake
        io.setJointSpeed(axis.getAsDouble());
    }

    public void raiseJointMotor(DoubleSupplier axis) { //This is supposed to raise the intake
        io.setJointSpeed(axis.getAsDouble());
    }

    public void reverseWheelMotor(DoubleSupplier axis) { //This will run the intake wheels backwards
        io.setWheelSpeed(axis.getAsDouble());
    }

    public void runWheelRollers(double value) { //This will run the intake wheels forward
        io.setWheelSpeed(value);
    }
    
    public void runWheel() { //This will run the intake wheels forward
        io.setWheelSpeed(.5);
    }

    public void raiseJoint() { //Bro I don't even know what this does
        io.setJointSpeed(.5);
    }

    public void reverseWheel() {//This will run the intake wheels backwards
        io.setWheelSpeed(-0.5);
    }

    public void lowerJoint() {
        io.setJointSpeed(-0.5);
    }

    public void runWheelMax() {
        io.setWheelSpeed(1.0);
    }

   
    public void stopJoint() { //In theory, this is supposed to stop the raising and lowering of the arm ig
        io.stopJoint();
    }

    public void stopWheel() { //This will stop the wheels from running
        io.stopWheel();
    }

   
}
