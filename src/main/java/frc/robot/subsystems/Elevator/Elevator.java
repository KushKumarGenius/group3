package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;


public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private final ElevatorIO io;
    private final PIDController pid;
    ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
    private double targetPosition = 0;
    

    public Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        queueState(ElevatorStates.IDLE);
        pid = new PIDController(1.0, 0.0, 0.0);
    }

    @Override
    public void outputPeriodic() {
        Logger.recordOutput("Elevator/ElevatorState", getState());
    }

    @Override
    public void handleStateMachine() {
        io.updateInputs(inputs);

        switch (getState()) {
            case MOVING_UP:
                if (inputs.atTop) {
                    queueState(ElevatorStates.IDLE);
                }
                else{
                    targetPosition = ElevatorConstants.ELEVATOR_MAX_HEIGHT;
                    moveUp();
                }
                break;
            case MOVING_DOWN:
                if (inputs.atBottom) {
                    queueState(ElevatorStates.IDLE);
                }
                else {
                    targetPosition = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
                    moveDown();
                }
                break;
            case IDLE:
                io.stopMoving();
                break;
        }
     
    }
    
    public void moveUp() {
        double volts = pid.calculate(inputs.elevatorPositionMeters, targetPosition);
        io.setMotorVoltage(volts);
        inputs.atTop = inputs.elevatorPositionMeters >= ElevatorConstants.ELEVATOR_MAX_HEIGHT;
        goUp();
    }

    public void moveDown() {
        double volts = pid.calculate(inputs.elevatorPositionMeters, targetPosition);
        io.setMotorVoltage(volts);
        inputs.atBottom = inputs.elevatorPositionMeters <= ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        goDown();
        
    }

    public void goUp() {
        queueState(ElevatorStates.MOVING_UP);
    }

    public void goDown() {
        queueState(ElevatorStates.MOVING_DOWN);
    }
}