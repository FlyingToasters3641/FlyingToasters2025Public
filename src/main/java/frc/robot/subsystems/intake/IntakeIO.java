package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {     
        
        public double jointPosition;
        public double jointVelocity;
        public double jointCurrent;
        public double wheelPosition;
        public double wheelVelocity;
        public double wheelCurrent;

    }

    public default void updateInputs(IntakeIOInputs inputs) {
      

    }

   
    public default void setJointSpeed(double speed){}
    
    public default void setWheelSpeed(double speed){}

    public default void stopJoint(){}

    public default void stopWheel(){}



}
