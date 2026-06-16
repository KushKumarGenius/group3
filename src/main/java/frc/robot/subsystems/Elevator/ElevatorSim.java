package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorSim implements ElevatorIO{
    private double position = 0;
    private double velocity = 0;
    private double motorVoltage = 0;

    private LinearSystem<N2, N1, N2> elevatorSystem;
    private final DCMotorSim elevatorMotorSim;
    
    public ElevatorSim(){
        elevatorSystem = LinearSystemId.createDCMotorSystem(1.0, 1.0);
        elevatorMotorSim = new DCMotorSim(elevatorSystem, DCMotor.getKrakenX60Foc(1), 1.0, 1.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        
        velocity += motorVoltage * 0.1; 
        position += velocity * 0.02; 

        inputs.elevatorPositionMeters = position;
        inputs.elevatorVelocityMetersPerSec = velocity;
        inputs.atTop = position >= ElevatorConstants.ELEVATOR_MAX_HEIGHT;
        inputs.atBottom = position <= ElevatorConstants.ELEVATOR_MIN_HEIGHT;
    }

    
}
