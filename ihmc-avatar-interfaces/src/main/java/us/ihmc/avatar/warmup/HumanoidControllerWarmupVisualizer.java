package us.ihmc.avatar.warmup;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class HumanoidControllerWarmupVisualizer
{
   public static void runAndVisualizeWarmup(HumanoidControllerWarmup controllerWarumup, FloatingRootJointRobot robot)
   {
      DoubleProvider timeProvider = controllerWarumup.getTimeProvider();
      FullRobotModel fullRobotModel = controllerWarumup.getFullRobotModel();
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerWarumup.getYoGraphicsListRegistry();
      YoVariableRegistry registry = controllerWarumup.getRegistry();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      plotterFactory.createOverheadPlotter();
      scs.setCameraTracking(true, true, true, true);
      scs.addYoVariableRegistry(registry);
      scs.setGroundVisible(false);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      scs.setTime(0.0);
      scs.tickAndUpdate();

      PerfectSimulatedOutputWriter writer = new PerfectSimulatedOutputWriter(robot, fullRobotModel);
      controllerWarumup.addTickListener(() -> {
         writer.updateRobotConfigurationBasedOnFullRobotModel();
         scs.setTime(timeProvider.getValue());
         scs.tickAndUpdate();
      });

      controllerWarumup.runWarmup();

      scs.setIndex(1);
      scs.setInPoint();
      scs.cropBuffer();
      scs.play();
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }
}
