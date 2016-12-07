package us.ihmc.exampleSimulations.skippy;

import us.ihmc.exampleSimulations.skippy.SkippyRobot.RobotType;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

public class SkippySimulation
{
   public static final double DT = 0.0001;
   public static final double controlDT = 0.0001;
   public static final double TIME = 20.0;
   public static final int recordFrequency = 75;

   private final SimulationConstructionSet sim;
   private final BlockingSimulationRunner blockingSimulationRunner;

   private static final boolean USE_ICP_BASED_CONTROLLER = true;

   public SkippySimulation()
   {
      // 0 - acrobot, 1 - skippy
      RobotType robotType = RobotType.SKIPPY;

      SkippyRobot skippy = new SkippyRobot(robotType);

      sim = new SimulationConstructionSet(skippy);
      sim.setGroundVisible(true);
      sim.setDT(DT, recordFrequency);
      sim.setMaxBufferSize(64000);
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

      blockingSimulationRunner = new BlockingSimulationRunner(sim, 10.0 * 60.0);
      sim.startOnAThread();
   }

   public boolean run(double simulationTime) throws SimulationExceededMaximumTimeException
   {
      return blockingSimulationRunner.simulateAndBlockAndCatchExceptions(simulationTime);
   }

   public void destroy()
   {
      blockingSimulationRunner.destroySimulation();
   }

   /*
    * When your simulation is run, first the main method will be called. In
    * creating a Skippy Simulation, a SkippyRobot is first created, and then a
    * Simulation Construction Set object is created with that robot. A Thread is
    * then created using the SimulationConstructionSet object. Finally the
    * Thread is started, thereby starting your simulation.
    */

   public static void main(String[] args) throws SimulationExceededMaximumTimeException
   {
      SkippySimulation skippySimulation = new SkippySimulation();
      skippySimulation.run(TIME);
      ThreadTools.sleepForever();
   }

}