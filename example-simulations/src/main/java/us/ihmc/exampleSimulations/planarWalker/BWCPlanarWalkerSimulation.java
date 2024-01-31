package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.scs2.SimulationConstructionSet2;

public class BWCPlanarWalkerSimulation
{
    private final SimulationConstructionSet2 scs = new SimulationConstructionSet2();

    public BWCPlanarWalkerSimulation()
    {
        scs.addRobot(new BWCPlanarWalkingRobotDefinition());

        scs.startSimulationThread();
        scs.simulate();
    }

    public static void main(String[] args)
    {
        new BWCPlanarWalkerSimulation();
    }

}
