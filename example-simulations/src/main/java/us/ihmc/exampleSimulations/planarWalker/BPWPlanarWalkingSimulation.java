package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.scs2.SimulationConstructionSet2;

public class BPWPlanarWalkingSimulation {

    public BPWPlanarWalkingSimulation()
    {
        SimulationConstructionSet2 scs = new SimulationConstructionSet2();
        scs.getGravity().setToZero();

        // Todo create a robot and add it to scs
        BPWPlanarWalkingRobotDefinition robotDefinition = new BPWPlanarWalkingRobotDefinition();
        scs.addRobot(robotDefinition);

        scs.startSimulationThread();
        scs.simulate();
    }


    public static void main(String[] args) {
        new BPWPlanarWalkingSimulation();
    }
}
