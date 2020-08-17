package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PeterPlanarWalkerSimulation
{
   private SimulationConstructionSet scs;
   PeterPlanarWalkerSimulation()
   {
      double simulationDT = 1e-4;
      
      PeterPlanarWalkerRobot robot = new PeterPlanarWalkerRobot();
      YoRegistry registry = robot.getRobotsYoRegistry();
      
      PeterPlanarWalkerController walkerController = new PeterPlanarWalkerController(robot, simulationDT);
//      FMSPeterPlanarWalkerController walkerController = new FMSPeterPlanarWalkerController(robot, simulationDT);
      robot.setController(walkerController);
    
      scs = new SimulationConstructionSet(robot);
      
      
      scs.setDT(simulationDT, 100);
      
      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
     new PeterPlanarWalkerSimulation();
   }
}
