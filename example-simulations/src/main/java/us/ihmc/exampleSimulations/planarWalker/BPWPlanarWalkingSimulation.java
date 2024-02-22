package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;

public class BPWPlanarWalkingSimulation {

    public BPWPlanarWalkingSimulation()
    {
        SimulationConstructionSet2 scs = new SimulationConstructionSet2();
//        scs.getGravity().setToZero();

        // Todo create a robot and add it to scs
        BPWPlanarWalkingRobotDefinition robotDefinition = new BPWPlanarWalkingRobotDefinition();
        Robot robot = new Robot(robotDefinition, scs.getInertialFrame());
        scs.addRobot(robot);

        scs.addTerrainObject(new SlopeGroundDefinition(0.0));

        // Set up the controller robot with some convenience method
        BPWPLanarWalkingRobot controllerRobot = new BPWPLanarWalkingRobot(robot);
        // controller
        BPWPlanarWalkingController controller = new BPWPlanarWalkingController(controllerRobot);
        // add the controller
        robot.addController(controller);

        scs.startSimulationThread();
        scs.simulate();
    }


    public static void main(String[] args) {
        new BPWPlanarWalkingSimulation();
    }
}
