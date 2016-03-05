package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 
 2 DOF Robot (2 legs)
 
    -- The robot starts out in a "splits" position 
    -- The rootJoint is now a FlotingPlanarJoint, which means the robot can translate in X and, hence, fall.
    -- We can control the body height by applying forces at the knees, while keeping the distance between the feet almost constant
    -- The body pitch is kept at 0.0 by applying torques at the hips.
    
    -- Note that:
       The feet slip slightly because when the robot changes from one body height to another, the robot state becomes slightly unstable
       for a short amount of time before reaching the next steady state. During that time, the feet bounce very slightly on the floor. 
       Each time the foot lifts off the floor, the leg will translate a little in X.
 
 **/

public class Step4Simulation
{

   //Variables 
   SimulationConstructionSet sim;
   private double deltaT = 0.0001;
   private int recordFrequency = 10;
   
   public Step4Simulation()
   {
      //Robot
      Step4Robot v4Robot = new Step4Robot();
      
      //Controller 
      Step4Controller v4Controller = new Step4Controller(v4Robot, "v4Robot", deltaT);
      v4Robot.setController(v4Controller);
      
      //Simulation Object  
      sim = new SimulationConstructionSet(v4Robot);
      sim.setGroundVisible(true);
      sim.setDT(deltaT, recordFrequency);
      sim.setCameraFix(0.0, 0.025, 1.05);
      sim.setCameraPosition(12.6, -15.4, 2.3);
      sim.changeBufferSize(32000);
      sim.selectConfiguration("Step4GUI.guiConf");

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new Step4Simulation();
   }

}
