package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
    private final String CANbusName = "lucas";
    public final TalonFX IN_JointTalonFX = new TalonFX(2, CANbusName); //This is for the Kraken motor that lowers and raises the intake
    public final TalonFX IN_WheelTalonFX = new TalonFX(1, CANbusName); //This is for the Kraken motor that spins the wheels



    public IntakeIOTalonFX() {
    
    
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
    }


    @Override
    public void setINAngle(double angle) {
    }
}
