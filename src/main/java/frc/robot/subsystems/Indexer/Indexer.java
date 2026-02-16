package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;
    private State state = State.OFF;

    private enum State {
        ON,
        OFF
    }
    
    public Indexer(IndexerIO indexerIO) {
        io = indexerIO;
        inputs = new IndexerIOInputsAutoLogged();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    /**
     * Run the indexer with a specified voltage. For use in 
     * @param voltage The voltage at which to run the indexer in volts
     * @return The command
     */
    public Command runIndexer(double voltage) {
        return new InstantCommand(() -> io.setVoltage(voltage), this);
    }

    /**
     * Toggles the indexer between on and off
     * @return The command to do this
     */
    public Command toggle () {
        return Commands.either(
            Commands.runOnce(() -> io.setVoltage(0)), // If on 
            Commands.runOnce(() -> io.setVoltage(IndexerConstants.MOTOR_VOLTAGE)), // if off
            () -> this.state == State.ON);
    }
}
