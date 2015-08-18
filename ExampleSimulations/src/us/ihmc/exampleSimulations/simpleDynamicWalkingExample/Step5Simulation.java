package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class Step5Simulation
{

   //Variables 
   SimulationConstructionSet sim;
   private double deltaT = 0.0001;
   private int recordFrequency = 10;
   
   public Step5Simulation()
   {
      //Robot
      Step5IDandSCSRobot v5Robot = new Step5IDandSCSRobot();
      
      //Controller 
      Step5Controller v5Controller = new Step5Controller(v5Robot, "v5Robot", deltaT);
      v5Robot.setController(v5Controller);
      
      //Simulation Object  
      sim = new SimulationConstructionSet(v5Robot);
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
      new Step5Simulation();
   }

}
