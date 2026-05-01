package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOSim implements IndexerIO {
    private double appliedVoltage = 0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.appliedVoltage = appliedVoltage;
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Indexer/Voltage", voltage);
        appliedVoltage = voltage;
    }
}
