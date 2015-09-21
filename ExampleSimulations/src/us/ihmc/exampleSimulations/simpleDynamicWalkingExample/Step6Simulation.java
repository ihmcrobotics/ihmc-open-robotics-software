package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class Step6Simulation
{

   //Variables 
   private SimulationConstructionSet sim;
   private double deltaT = 0.0001;
   private int recordFrequency = 10;
   
   public Step6Simulation()
   {
      //Robot
      Step6IDandSCSRobot walkingRobot = new Step6IDandSCSRobot();
      
      //Controller 
      Step6WalkingController_SpringFlamInspired walkingController = new Step6WalkingController_SpringFlamInspired(walkingRobot, "v5Robot", deltaT);
      walkingRobot.setController(walkingController);
      
      //Simulation Object  
      sim = new SimulationConstructionSet(walkingRobot);
      sim.setGroundVisible(true);
      sim.setDT(deltaT, recordFrequency);
      sim.setCameraFix(0.0, 0.025, 1.05);
      sim.setCameraPosition(12.6, -15.4, 2.3);
      sim.changeBufferSize(32000);
      sim.selectConfiguration("Step5GUI.guiConf");

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new Step6Simulation();
   }

}
