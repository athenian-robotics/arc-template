package frc.robot.subsystems.Hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs{
        public double hopperExtension_rotations = 0;
        public double hopperExtension_inches = 0;
        public double hopperExtensionVelocity_inchesPerSecond = 0;
        public double hopperMotor_Volts = 0;
        public double hopperMotor_Amps = 0;
        //public double hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
        
    }
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs);
    public void positionToRotations(double position);
    public void goToPosition(double position);
    

    

        

        // Constants (TODO move)
}
