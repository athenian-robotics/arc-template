package frc.robot.subsystems.hopintake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Revolutions;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DIOSim;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Hopper {
    private final static int SPARK_ID = 0;
    private final static int WINCH_GEARBOX_GEAR_RATIO = 10;
    private final static Distance WINCH_DIAMETER = Inches.of(0.75);

    private final static Distance SETPOINT_RETRACTED = Inches.of(0);
    private final static Distance SETPOINT_MOVE_INTAKE = Inches.of(4);
    private final static Distance SETPOINT_EXTENDED = Inches.of(11.425);

    private Distance extension = SETPOINT_RETRACTED;
            
    private final SparkMax motor = new SparkMax(SPARK_ID, MotorType.kBrushless);
            
    private static Angle lenToMotorAngle(Distance len) {
        return Revolutions.of(len.in(Inches) * WINCH_GEARBOX_GEAR_RATIO / (WINCH_DIAMETER.in(Inches) * Math.PI));
    }

    public void moveTo(Distance target) {
        
    }
}
