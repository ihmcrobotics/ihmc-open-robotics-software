package us.ihmc.exampleSimulations.skippy;

import us.ihmc.exampleSimulations.skippy.SkippyRobot.RobotType;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;

public class SkippySimulation
{
   public static final double DT = 0.0001;
   public static final double controlDT = 0.0001;
   public static final double TIME = 20.0;
   private SimulationConstructionSet sim;

   private static final boolean USE_ICP_BASED_CONTROLLER = false;//true;//     

   public SkippySimulation()
   {
      // 0 - acrobot, 1 - skippy
      RobotType robotType = RobotType.SKIPPY;

      SkippyRobot skippy = new SkippyRobot(robotType);

      sim = new SimulationConstructionSet(skippy);
      sim.setGroundVisible(true);
      sim.setDT(DT, 75);
      sim.setMaxBufferSize(128000);
      sim.setSimulateDuration(TIME);
      sim.setCameraPosition(10.0, 0.0, 2.0);

      boolean showOverheadView = true;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      RobotController controller = null;
      if (USE_ICP_BASED_CONTROLLER)
         controller = new SkippyICPBasedController(skippy, controlDT, yoGraphicsListRegistry);
      else
         controller = new SkippyController(skippy, robotType, "skippyController", controlDT, yoGraphicsListRegistry);

      skippy.setController(controller);
      // skippy.setController(new ExternalControlServer(skippy,
      // "externalControlServer"));

      VisualizerUtils.createOverheadPlotter(sim, showOverheadView, yoGraphicsListRegistry);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      Thread myThread = new Thread(sim);
      myThread.start();
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