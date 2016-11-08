package us.ihmc.exampleSimulations.skippy;

import us.ihmc.exampleSimulations.skippy.SkippyRobot.RobotType;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class SkippySimulation
{
   public static final double DT = 0.0001;
   public static final double controlDT = 0.0001;
   public static final double TIME = 3.5;//8.0;//10.0;//15.0;//25.0;//3.0;//5.0;//2.5;//60.0;// 
   private static SimulationConstructionSet sim;

   public SkippySimulation()
   {
      // 0 - acrobot, 1 - skippy
      RobotType robotType = RobotType.SKIPPY;

      SkippyRobot skippy = new SkippyRobot(robotType);

      sim = new SimulationConstructionSet(skippy);
      sim.setGroundVisible(true);
      sim.setDT(DT, 10);
      sim.setSimulateDuration(TIME);
      sim.setCameraPosition(10.0,   0.0, 2.0);

      boolean showOverheadView = true;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SkippyController skippyController = new SkippyController(skippy, robotType, "skippyController", controlDT, yoGraphicsListRegistry);
      skippy.setController(skippyController);
      // skippy.setController(new ExternalControlServer(skippy,
      // "externalControlServer"));

      VisualizerUtils.createOverheadPlotter(sim, showOverheadView, yoGraphicsListRegistry);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      Thread myThread = new Thread(sim);
      myThread.start();
      try
      {
         myThread.join();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
      skippyController.closeFile();

   }

   /*
    * When your simulation is run, first the main method will be called. In
    * creating a Skippy Simulation, a SkippyRobot is first created, and then a
    * Simulation Construction Set object is created with that robot. A Thread is
    * then created using the SimulationConstructionSet object. Finally the
    * Thread is started, thereby starting your simulation.
    */

   public static void main(String[] args)
   {
      new SkippySimulation();
   }
}