package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 
 1 DOF robot  (1 leg)
  
    -- Robot only moves up and down through application of forces at the kneeJoint 
    -- We can control the desired height of the rootJoint (the body)
    -- The lowerLink gets "sucked in" (it lifts off the ground) because the controller 
       tells it to pull down on the upperLink and when it does, it experiences a reaction
       force upwards that is greater than the force of gravity pulling it down.
    -- The "sucking effect" can be solved by tweaking the gains as we will see in Step2
    
 */

public class Step0Simulation
{
   SimulationConstructionSet sim;

   public Step0Simulation()
   {
      double deltaT = 0.00001; // Simulation step size in seconds

      //Robot
      Step0Robot v0Robot = new Step0Robot();

      //Controller
      Step0Controller v0Controller = new Step0Controller(v0Robot, "v0Robot", deltaT);
      v0Robot.setController(v0Controller);

      //Simulation object
      sim = new SimulationConstructionSet(v0Robot);
      sim.setGroundVisible(true);
      sim.setDT(deltaT, 10);

      sim.setCameraFix(0.0, 0.025, 1.05);
      sim.setCameraPosition(0.0, -20.0, 2.0);

      sim.changeBufferSize(32000);
      
      sim.selectConfiguration("Step0GUI.guiConf"); //makes sure that the window configuration is always the same. You can change it in the simulation window: configuration > save configuration
      
      Thread myThread = new Thread(sim);
      myThread.start();
     
   }

   //Main
   public static void main(String[] args)
   {
      new Step0Simulation();
   }

}
