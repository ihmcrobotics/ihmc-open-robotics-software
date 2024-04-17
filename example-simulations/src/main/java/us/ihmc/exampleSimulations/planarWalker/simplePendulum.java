package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;

public class simplePendulum {

    public simplePendulum()
    {
        int simTicksPerControlTick = 3;

        SimulationConstructionSet2 scs = new SimulationConstructionSet2();

        simplePendulumDefinition pendulum = new simplePendulumDefinition();
        Robot robot = new Robot(pendulum, scs.getInertialFrame());
        scs.addRobot(robot);

        simplePendulumRobot pendulumRobot = new simplePendulumRobot(robot);
        simplePendulumController pendulumController = new simplePendulumController(pendulumRobot);
        robot.addThrottledController(pendulumController , scs.getDT() * simTicksPerControlTick);


        scs.startSimulationThread();
        scs.simulate();
    }

    public static void main(String[] args) {
        new simplePendulum();
    }
}
