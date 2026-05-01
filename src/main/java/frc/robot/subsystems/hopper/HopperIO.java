package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.HopperConstants;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs{
        public double hopperExtension_Rotations = 0;
        public double hopperExtension_Inches = (hopperExtension_Rotations  / HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
        public double hopperExtensionVelocity_inchesPerSecond = 0;
        public double hopperMotor_Volts = 0;
        public double hopperMotor_Amps = 0;
        //public double hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
        
    }
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs);
    /**
     * Goes the hopper to position 
     * @param position in inches
     */
    public void goToPosition(double position);
    

    

        

        // Constants (TODO move)
}
