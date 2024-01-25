package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.scs2.SimulationConstructionSet2;

public class BWCPlanarWalkingSimulation
{

   public BWCPlanarWalkingSimulation ()
   {
      SimulationConstructionSet2 scs = new SimulationConstructionSet2();

      scs.getGravity().set(0.0, 0.0, 0.0);

      BWCPlanarWalkingRobotDefinition robotDefinition = new BWCPlanarWalkingRobotDefinition();
      scs.addRobot(robotDefinition);

      //TODO create ground

      scs.startSimulationThread();
      scs.setShutdownSessionOnVisualizerClose(true);
   }

   public static void main(String[] args)
   {
      new BWCPlanarWalkingSimulation();
   }
}
