package us.ihmc.exampleSimulations.skippy;

import us.ihmc.exampleSimulations.skippy.SkippyRobot.RobotType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
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
   private final SkippyRobot skippy;

   public enum SkippyControllerMode
   {
      ICP_BASED,
      STATE_FEEDBACK,
      NONE
   }

   public SkippySimulation(SkippyControllerMode controllerMode)
   {
      // 0 - acrobot, 1 - skippy
      RobotType robotType = RobotType.SKIPPY;
      skippy = new SkippyRobot(robotType);

      sim = new SimulationConstructionSet(skippy);
      sim.setGroundVisible(true);
      sim.setDT(DT, recordFrequency);
      sim.setMaxBufferSize(64000);
      sim.setCameraPosition(10.0, 0.0, 2.0);

      boolean showOverheadView = true;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      switch (controllerMode)
      {
      case ICP_BASED:
         SkippyICPBasedController icpBasedController = new SkippyICPBasedController(skippy, controlDT, yoGraphicsListRegistry);
         skippy.setController(icpBasedController);
         break;
      case STATE_FEEDBACK:
         SkippyController stateFeedbackController = new SkippyController(skippy, robotType, "skippyController", controlDT, yoGraphicsListRegistry);
         skippy.setController(stateFeedbackController);
         break;
      case NONE:
      default:
         break;
      }

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = sim.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      blockingSimulationRunner = new BlockingSimulationRunner(sim, 10.0 * 60.0);

      ControllerFailureListener controllerFailureListener = blockingSimulationRunner.createControllerFailureListener();
      skippy.setController(new SkippyFallingChecker(controllerFailureListener, skippy));

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

   public SkippyRobot getSkippy()
   {
      return skippy;
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
      SkippySimulation skippySimulation = new SkippySimulation(SkippyControllerMode.ICP_BASED);
      skippySimulation.run(TIME);
      ThreadTools.sleepForever();
   }

}