package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PeterPlanarWalkerSimulation
{
   private SimulationConstructionSet scs;
   PeterPlanarWalkerSimulation()
   {
      double simulationDT = 1e-4;
      
      PeterPlanarWalkerRobot robot = new PeterPlanarWalkerRobot();
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      
      PeterPlanarWalkerController walkerController = new PeterPlanarWalkerController(robot, simulationDT);
//      WDIPeterPlanarWalkerController walkerController = new WDIPeterPlanarWalkerController(robot, simulationDT);
      robot.setController(walkerController);
    
      scs = new SimulationConstructionSet(robot);
      
      
      scs.setDT(simulationDT, 100);
      
      scs.setMaxBufferSize(32000);
      
      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
     new PeterPlanarWalkerSimulation();
   }
}
