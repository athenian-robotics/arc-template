package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase{
            
    private HopperIO io;
    private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }
    public void retract(){
        io.goToPosition(HopperConstants.HOPPER_RETRACTED * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
    }
    public void partial(){
        io.goToPosition(HopperConstants.HOPPER_PARTIAL * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
    }
    public void full(){
        io.goToPosition(HopperConstants.HOPPER_FULL * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
    }
   

}
