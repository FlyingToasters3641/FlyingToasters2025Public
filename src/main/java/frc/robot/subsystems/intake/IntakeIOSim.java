package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
    private double jointSpeed = 0.0;
    private double wheelSpeed = 0.0;
    private double jointPosition = 0.0;
    private double wheelPosition = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Simulate joint and wheel positions/velocities ig
        jointPosition += jointSpeed * 0.02; //20ms?
        wheelPosition += wheelSpeed * 0.02;

        inputs.jointPosition = jointPosition;
        inputs.jointVelocity = jointSpeed;
        inputs.wheelPosition = wheelPosition;
        inputs.wheelVelocity = wheelSpeed;

        inputs.jointCurrent = Math.abs(jointSpeed) * 10;
        inputs.wheelCurrent = Math.abs(wheelSpeed) * 10;
    }

    @Override
    public void setJointSpeed(double speed) {
        jointSpeed = speed;
    }

    @Override
    public void setWheelSpeed(double speed) {
        wheelSpeed = speed;
    }

    @Override
    public void stopJoint() {
        jointSpeed = 0.0;
    }

    @Override
    public void stopWheel() {
        wheelSpeed = 0.0;
    }
}