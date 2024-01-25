package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.scs2.SimulationConstructionSet2;

public class BWCPlanarWalkerSimulation
{
   public BWCPlanarWalkerSimulation()
   {
      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      scs.getGravity().setToZero();

      BWCPlanarWalkingRobotDefinition robotDefinition = new BWCPlanarWalkingRobotDefinition();
      scs.addRobot(robotDefinition);
      // TODO create ground
      // TODO create a robot controller and add it to the robot

      scs.startSimulationThread();
      scs.simulate();
   }

   public static void main(String[] args)
   {
      new BWCPlanarWalkerSimulation();
   }
}
