package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;


public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private final ElevatorIO io;
    private final ElevatorSim sim;
    

    public Elevator(ElevatorIO io, ElevatorSim sim) {
        super("Elevator");
        this.io = io;
        this.sim = sim;
        queueState(ElevatorStates.IDLE);
    }

    public void update() {
        ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
        io.updateInputs(inputs);

        switch (getState()) {
            case MOVING_UP:
                if (inputs.atTop) {
                    queueState(ElevatorStates.IDLE);
                    io.stopMoving();
                }
                break;
            case MOVING_DOWN:
                if (inputs.atBottom) {
                    queueState(ElevatorStates.IDLE);
                    io.stopMoving();
                }
                break;
            case IDLE:
                // Do nothing
                break;
        }
    }

    public void handleStateMachine(){
     
    }

    @Override
    public void outputPeriodic() {
        // Logger.recordOutput("Intake/moveMotorDegps", inputs.flyVel_rps);
        Logger.recordOutput("Elevator/ElevatorState", getState());
    }
    public void moveUp() {
        queueState(ElevatorStates.MOVING_UP);
        io.setElevatorMotorVoltage(12.0); // Example voltage to move up
    }

    public void moveDown() {
        queueState(ElevatorStates.MOVING_DOWN);
        io.setElevatorMotorVoltage(-12.0); // Example voltage to move down
    }

    public void stop() {
        queueState(ElevatorStates.IDLE);
        io.stopMoving();
    }
}