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
        io.goToPosition(positionToRotations(HopperConstants.HOPPER_RETRACTED));
    }
    public void partial(){
        io.goToPosition(positionToRotations(HopperConstants.HOPPER_PARTIAL));
    }
    public void full(){
        io.goToPosition(positionToRotations(HopperConstants.HOPPER_FULL));
    }
   

}
