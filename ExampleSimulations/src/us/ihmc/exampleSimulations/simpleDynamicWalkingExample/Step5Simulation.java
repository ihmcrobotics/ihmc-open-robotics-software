package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class Step5Simulation
{

   //Variables 
   private SimulationConstructionSet sim;
   private double deltaT = 1e-4;
   private int recordFrequency = 80;
   
   public Step5Simulation()
   {
      //Robot
      Step5IDandSCSRobot_pinKnee v5Robot = new Step5IDandSCSRobot_pinKnee();
      
      //Controller 
      Step5WalkingController v5Controller = new Step5WalkingController(v5Robot, "v5Robot", deltaT);
      v5Robot.setController(v5Controller);
      
      //Simulation Object  
      sim = new SimulationConstructionSet(v5Robot);
      sim.setGroundVisible(true);
      sim.setDT(deltaT, recordFrequency);
      sim.setCameraFix(0.0, 0.025, 1.05);
      sim.setCameraPosition(0.8643, -17.188, 2.72);
      sim.changeBufferSize(32000);
      sim.selectConfiguration("step5Config.guiConf");

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new Step5Simulation();
   }

}
