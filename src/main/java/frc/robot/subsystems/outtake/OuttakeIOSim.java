package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;

public class OuttakeIOSim implements OuttakeIO {

    @Override
    public void updateInputs(OuttakeIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void startFlywheel() {
        Logger.recordOutput("Outtake/FlywheelVoltage", 12);
    }

    @Override
    public void stopFlywheel() {
        Logger.recordOutput("Outtake/FlywheelVoltage", 0);
    }

    @Override
    public void setMiddleWheelVoltage(double voltage) {
        Logger.recordOutput("Outtake/MiddleWheelVoltage", voltage);
    }

    @Override
    public void setStarWheelVoltage(double voltage) {
        Logger.recordOutput("Outtake/StarWheelVoltage", voltage);
    }

    @Override
    public void setAngleAtTarget(Translation2d currentPosition) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngleAtTarget'");
    }

    @Override
    public void setAngle(double angleDegrees) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngle'");
    }

    @Override
    public void setAngleFromNT() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngleFromNT'");
    }
    
}
