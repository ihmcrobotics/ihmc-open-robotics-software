package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class Step6Simulation
{

   /**
    *  Walking robot with an ICP visualizer
    */
   
   //Variables 
   private SimulationConstructionSet sim;
   private double deltaT = 1e-4;
   private int recordFrequency = 120;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   
   public Step6Simulation()
   {
      //Robot
      Step6IDandSCSRobot_pinKnee v6Robot = new Step6IDandSCSRobot_pinKnee();
      
      //Controller 
      Step6WalkingController v6Controller = new Step6WalkingController(v6Robot, "v6Robot", deltaT, yoGraphicsListRegistry);
      v6Robot.setController(v6Controller);
      
      //Simulation Object        
      sim = new SimulationConstructionSet(v6Robot);
      sim.setGroundVisible(true);
      sim.setDT(deltaT, recordFrequency);
      sim.setCameraFix(0.0, 0.025, 1.05);
      sim.setCameraPosition(0.8643, -17.188, 2.72);
      sim.changeBufferSize(16000);
      sim.selectConfiguration("step6Config.guiConf");
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      sim.startOnAThread();
      sim.simulate();
   }

   public static void main(String[] args)
   {
      new Step6Simulation();
   }

}
