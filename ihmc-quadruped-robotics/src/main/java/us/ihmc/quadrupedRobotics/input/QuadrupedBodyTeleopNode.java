package us.ihmc.quadrupedRobotics.input;

import controller_msgs.msg.dds.RobotConfigurationData;
import net.java.games.input.Event;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.mode.QuadrupedStepTeleopMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.sensorProcessing.communication.subscribers.RobotDataReceiver;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

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

   private final QuadrupedStepTeleopMode stepTeleopMode;
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   public QuadrupedBodyTeleopNode(String host, NetworkPorts port, NetClassList netClassList, Joystick device,
                                  FullQuadrupedRobotModel fullRobotModel, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                  QuadrupedPhysicalProperties physicalProperties) throws IOException
   {
      this.device = device;

      this.server = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadSchedulerFactory(), null, LogSettings.BEHAVIOR, DT);
      this.server.setMainRegistry(registry, fullRobotModel.getElevator(), graphicsListRegistry);
      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, port, netClassList);
      this.robotDataReceiver = new RobotDataReceiver(fullRobotModel, null);
      this.packetCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      this.stepTeleopMode = new QuadrupedStepTeleopMode(packetCommunicator, physicalProperties, defaultXGaitSettings, referenceFrames, graphicsListRegistry, registry);

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
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_TRIGGER, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_TRIGGER, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_Y, true, 0.1, 1));
   }

   public void update()
   {
      robotDataReceiver.updateRobotModel();

      stepTeleopMode.update(Collections.unmodifiableMap(channels));

      server.update(robotDataReceiver.getSimTimestamp());
   }

   @Override
   public void processEvent(Event event)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(XBoxOneMapping.getMapping(event), (double) event.getValue());

      // Handle events that should trigger once immediately after the event is triggered.
      stepTeleopMode.onInputEvent(Collections.unmodifiableMap(channels), event);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
