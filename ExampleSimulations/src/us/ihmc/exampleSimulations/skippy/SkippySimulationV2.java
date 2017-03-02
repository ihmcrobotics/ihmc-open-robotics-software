package us.ihmc.exampleSimulations.skippy;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

public class SkippySimulationV2
{
   public static final double DT = 0.0001;
   public static final double controlDT = 0.0001;
   public static final double TIME = 20.0;
   public static final int recordFrequency = 75;

   private final SimulationConstructionSet sim;
   private final BlockingSimulationRunner blockingSimulationRunner;
   private final ControllerFailureListener controllerFailureListener;
   private final SkippyRobotV2 skippy;

   public SkippySimulationV2()
   {
      skippy = new SkippyRobotV2();

      sim = new SimulationConstructionSet(skippy);
      sim.setGroundVisible(true);
      sim.setDT(DT, recordFrequency);
      sim.setMaxBufferSize(64000);
      sim.setCameraPosition(10.0, 0.0, 2.0);

      boolean showOverheadView = true;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      skippy.setController(new SkippyICPAndIDBasedController(skippy, yoGraphicsListRegistry));

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = sim.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      blockingSimulationRunner = new BlockingSimulationRunner(sim, 10.0 * 60.0);
      controllerFailureListener = blockingSimulationRunner.createControllerFailureListener();
      skippy.setController(new FallChecker());

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

   public SkippyRobotV2 getSkippy()
   {
      return skippy;
   }

   public static void main(String[] args) throws SimulationExceededMaximumTimeException
   {
      SkippySimulationV2 skippySimulation = new SkippySimulationV2();
      skippySimulation.run(TIME);
      ThreadTools.sleepForever();
   }

   private class FallChecker extends SimpleRobotController
   {
      @Override
      public void doControl()
      {
         Point3D footLocation = skippy.getFootLocation();
         Point3D com = new Point3D();
         skippy.computeCenterOfMass(com);

         boolean skippyFalling = com.getZ() < footLocation.getZ() + 0.1;
         if (skippyFalling)
            controllerFailureListener.controllerFailed(null);
      }
   }

}