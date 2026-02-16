package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public double appliedVoltage = 0;
    }
    
    // Updates the logged inputs
    public default void updateInputs (IndexerIOInputs inputs) {}

    // Runs the indexer at the specified voltage
    public default void setVoltage (double voltage) {}
}