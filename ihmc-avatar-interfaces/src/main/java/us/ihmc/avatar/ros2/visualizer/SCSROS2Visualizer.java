package us.ihmc.avatar.ros2.visualizer;

import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.log.LogTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import javax.swing.*;

/**
 * Debug tool that plots ROS 2 discovery statistics in Simulation Construction Set.
 * Participant and endpoint discovery counting is not yet exposed by jros2; the YoVariables remain for API compatibility.
 */
public class SCSROS2Visualizer
{
   private ROS2Node ros2Node;
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Robot"),
                                                                               new NullGraphics3DAdapter(),
                                                                               new SimulationConstructionSetParameters());
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final YoDouble timeElapsed = new YoDouble("timeElapsed", yoRegistry);
   private final YoLong numberOfParticipants = new YoLong("numberOfParticipants", yoRegistry);
   private final YoLong numberOfPublishers = new YoLong("numberOfPublishers", yoRegistry);
   private final YoLong numberOfSubscribers = new YoLong("numberOfSubscribers", yoRegistry);
   private final YoLong numberOfEndpoints = new YoLong("numberOfEndpoints", yoRegistry);
   private final PausablePeriodicThread updateThread;
   private boolean paused = false;

   public SCSROS2Visualizer()
   {
      ExceptionTools.handle(this::setupROS2Debugger, DefaultExceptionHandler.PRINT_STACKTRACE);

      scs.addYoRegistry(yoRegistry);
      scs.setDT(1.0, 1);
      scs.setupGraph(timeElapsed.getName());
      scs.setupGraph(numberOfParticipants.getName());
      scs.setupGraph(numberOfPublishers.getName());
      scs.setupGraph(numberOfSubscribers.getName());
      scs.setupGraph(numberOfEndpoints.getName());
      scs.skipLoadingDefaultConfiguration();
      scs.hideViewport();
      scs.changeBufferSize(200);
      scs.setScrollGraphsEnabled(false);
      scs.getGUI().getFrame().setSize(1200, 800);
      JToggleButton pauseButton = new JToggleButton("Pause");
      pauseButton.addActionListener(e -> {
         paused = pauseButton.isSelected();
         scs.setScrollGraphsEnabled(paused);
      });
      scs.addButton(pauseButton);
      scs.startOnAThread();
      while (!scs.hasSimulationThreadStarted())
         ThreadTools.sleep(200);

      updateThread = new PausablePeriodicThread(getClass().getSimpleName(), UnitConversions.hertzToSeconds(5.0), this::update);
      updateThread.start();
   }

   private void setupROS2Debugger()
   {
      ros2Node = new ROS2Node(getClass().getSimpleName());
      LogTools.info("Created jros2 node for SCS ROS 2 visualizer on domain {}", ros2Node.getDomainId());
      // JROS2_TODO: RTPS participant/endpoint discovery counting is not exposed by jros2 yet; YoVariables stay at zero.
      LogTools.warn("RTPS participant and endpoint discovery counting is not yet available in jros2");
   }

   private void update()
   {
      timeElapsed.set(stopwatch.totalElapsed());

      if (!paused)
      {
         int bufferSize = scs.getDataBuffer().getBufferSize();
         if (scs.getDataBuffer().getCurrentIndex() == bufferSize - 2)
         {
            scs.changeBufferSize(bufferSize + bufferSize / 2);
         }
         scs.tickAndUpdate();
      }
   }

   public static void main(String[] args)
   {
      new SCSROS2Visualizer();
   }
}
