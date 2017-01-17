package us.ihmc.quadrupedRobotics.input;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import net.java.games.input.Event;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.mode.QuadrupedStepTeleopMode;
import us.ihmc.quadrupedRobotics.input.mode.QuadrupedTeleopMode;
import us.ihmc.quadrupedRobotics.input.mode.QuadrupedXGaitTeleopMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.subscribers.RobotDataReceiver;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickComponentFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

public class QuadrupedBodyTeleopNode implements JoystickEventListener
{
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private final Joystick device;
   private final Map<XBoxOneMapping, Double> channels = Collections.synchronizedMap(new EnumMap<XBoxOneMapping, Double>(XBoxOneMapping.class));
   private final ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();

   private final YoVariableServer server;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PacketCommunicator packetCommunicator;
   private final RobotDataReceiver robotDataReceiver;
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();

   private final QuadrupedXGaitTeleopMode xGaitTeleopMode;
   private final QuadrupedStepTeleopMode stepTeleopMode;
   private QuadrupedTeleopMode activeTeleopMode;

   public QuadrupedBodyTeleopNode(String host, NetworkPorts port, NetClassList netClassList, Joystick device,
         FullQuadrupedRobotModel fullRobotModel, QuadrupedPhysicalProperties physicalProperties) throws IOException
   {
      this.device = device;

      this.server = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName()), null, LogSettings.BEHAVIOR, DT);
      this.server.setMainRegistry(registry, fullRobotModel, new YoGraphicsListRegistry());
      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, port, netClassList);
      this.robotDataReceiver = new RobotDataReceiver(fullRobotModel, null);
      this.packetCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      this.taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(fullRobotModel, referenceFrames, registry, null);
      this.xGaitTeleopMode = new QuadrupedXGaitTeleopMode(packetCommunicator, referenceFrames);
      this.stepTeleopMode = new QuadrupedStepTeleopMode(packetCommunicator, referenceFrames);

      // Set the default teleop mode.
      this.activeTeleopMode = xGaitTeleopMode;

      // Initialize all channels to zero.
      for (XBoxOneMapping channel : XBoxOneMapping.values)
      {
         channels.put(channel, 0.0);
      }
   }

   public void start() throws IOException, InterruptedException
   {
      packetCommunicator.connect();
      server.start();

      // Poll the data receiver until the first packet has been received and frames are properly initialized.
      // TODO: Polling isn't the best solution.
      while (!robotDataReceiver.framesHaveBeenSetUp())
      {
         robotDataReceiver.updateRobotModel();
         Thread.sleep(10);
      }

      activeTeleopMode.onEntry();

      // Send packets and integrate at fixed interval.
      executor.scheduleAtFixedRate(new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               update();
            }
            catch (Exception e)
            {
               e.printStackTrace();
               executor.shutdown();
            }
         }
      }, 0, (long) (DT * 1000), TimeUnit.MILLISECONDS);

      configureJoystickFilters(device);
      device.addJoystickEventListener(this);
      device.setPollInterval(10);
   }

   private void configureJoystickFilters(Joystick device)
   {
      device.setComponentFilter(new JoystickComponentFilter(XBoxOneMapping.LEFT_TRIGGER, false, 0.05, 1, 1.0));
      device.setComponentFilter(new JoystickComponentFilter(XBoxOneMapping.RIGHT_TRIGGER, false, 0.05, 1, 1.0));
      device.setComponentFilter(new JoystickComponentFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1, 1));
      device.setComponentFilter(new JoystickComponentFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1, 1));
      device.setComponentFilter(new JoystickComponentFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1, 1));
      device.setComponentFilter(new JoystickComponentFilter(XBoxOneMapping.RIGHT_STICK_Y, true, 0.1, 1));
   }

   public void update()
   {
      robotDataReceiver.updateRobotModel();
      taskSpaceEstimator.compute(taskSpaceEstimates);

      activeTeleopMode.update(Collections.unmodifiableMap(channels), taskSpaceEstimates);

      server.update(robotDataReceiver.getSimTimestamp());
   }

   @Override
   public void processEvent(Event event)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(XBoxOneMapping.getMapping(event), (double) event.getValue());

      // Handle events that should trigger once immediately after the event is triggered.
      switch (XBoxOneMapping.getMapping(event))
      {
      case SELECT:
         activeTeleopMode.onExit();
         activeTeleopMode = stepTeleopMode;
         activeTeleopMode.onEntry();
         break;
      case START:
         activeTeleopMode.onExit();
         activeTeleopMode = xGaitTeleopMode;
         activeTeleopMode.onEntry();
         break;
      default:
         // Pass any non-mode-switching events down to the active mode.
         activeTeleopMode.onInputEvent(Collections.unmodifiableMap(channels), taskSpaceEstimates, event);
      }
   }
}
