package us.ihmc.quadrupedRobotics.input;

import controller_msgs.msg.dds.RobotConfigurationData;
import net.java.games.input.Event;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedRobotics.communication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.input.value.InputValueIntegrator;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.PointFootSnapper;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.subscribers.RobotDataReceiver;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class QuadrupedXBoxAdapter implements JoystickEventListener
{
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer server;

   private final Joystick device;
   private final Map<XBoxOneMapping, Double> channels = Collections.synchronizedMap(new EnumMap<XBoxOneMapping, Double>(XBoxOneMapping.class));
   private final ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();

   private final RobotDataReceiver robotDataReceiver;

   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final QuadrupedTeleopManager stepTeleopManager;

   private final YoDouble maxBodyYaw = new YoDouble("maxBodyYaw", registry);
   private final YoDouble maxBodyPitch = new YoDouble("maxBodyPitch", registry);
   private final YoDouble maxBodyHeightVelocity = new YoDouble("maxBodyHeightVelocity", registry);
   private final YoDouble maxVelocityX = new YoDouble("maxVelocityX", registry);
   private final YoDouble maxVelocityY = new YoDouble("maxVelocityY", registry);
   private final YoDouble maxVelocityYaw = new YoDouble("maxVelocityYaw", registry);
   private final YoDouble bodyOrientationShiftTime = new YoDouble("bodyOrientationShiftTime", registry);
   private InputValueIntegrator bodyHeight;

   public QuadrupedXBoxAdapter(String robotName, Joystick device, FullQuadrupedRobotModel fullRobotModel, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                  QuadrupedPhysicalProperties physicalProperties)
   {
      this.device = device;

      this.server = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadSchedulerFactory(), null, LogSettings.BEHAVIOR, DT);
      this.server.setMainRegistry(registry, fullRobotModel.getElevator(), graphicsListRegistry);
      this.robotDataReceiver = new RobotDataReceiver(fullRobotModel, null);

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "quadruped_teleop_node");
      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> robotDataReceiver.receivedPacket(s.takeNextData()));

      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      this.stepTeleopManager = new QuadrupedTeleopManager(robotName, ros2Node, defaultXGaitSettings, physicalProperties.getNominalCoMHeight(), referenceFrames,
                                                          DT, graphicsListRegistry, registry);

      maxBodyYaw.set(0.15);
      maxBodyPitch.set(0.15);
      maxBodyHeightVelocity.set(0.1);
      maxVelocityX.set(0.5);
      maxVelocityY.set(0.25);
      maxVelocityYaw.set(0.4);
      bodyOrientationShiftTime.set(0.1);
      this.bodyHeight = new InputValueIntegrator(DT, physicalProperties.getNominalCoMHeight());

      // Initialize all channels to zero.
      for (XBoxOneMapping channel : XBoxOneMapping.values)
      {
         channels.put(channel, 0.0);
      }
   }

   public void start() throws InterruptedException
   {
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

      if (device != null)
      {
         configureJoystickFilters(device);
         device.addJoystickEventListener(this);
         device.setPollInterval(10);
      }
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

      if (device != null)
      {
         processJoystickCommands();
      }

      stepTeleopManager.update();
      server.update(robotDataReceiver.getSimTimestamp());
   }

   private void processJoystickCommands()
   {
      double bodyRoll = 0.0;
      double bodyPitch = channels.get(XBoxOneMapping.RIGHT_STICK_Y) * maxBodyPitch.getValue();
      double bodyYaw = channels.get(XBoxOneMapping.RIGHT_STICK_X) * maxBodyYaw.getValue();
      stepTeleopManager.setDesiredBodyOrientation(bodyYaw, bodyPitch, bodyRoll, bodyOrientationShiftTime.getValue());

      if (channels.get(XBoxOneMapping.DPAD) == 0.25)
      {
         double bodyHeightVelocity = maxBodyHeightVelocity.getValue();
         stepTeleopManager.setDesiredBodyHeight(bodyHeight.update(bodyHeightVelocity));
      }
      else if (channels.get(XBoxOneMapping.DPAD) == 0.75)
      {
         double bodyHeightVelocity = - maxBodyHeightVelocity.getValue();
         stepTeleopManager.setDesiredBodyHeight(bodyHeight.update(bodyHeightVelocity));
      }

      double xVelocity = channels.get(XBoxOneMapping.LEFT_STICK_Y) * maxVelocityX.getDoubleValue();
      double yVelocity = channels.get(XBoxOneMapping.LEFT_STICK_X) * maxVelocityY.getDoubleValue();
      double yawRate = channels.get(XBoxOneMapping.RIGHT_STICK_X) * maxVelocityYaw.getDoubleValue();
      stepTeleopManager.setDesiredVelocity(xVelocity, yVelocity, yawRate);
   }

   private void processStateChangeRequests(Event event)
   {
      if (event.getValue() < 0.5)
         return;

      if (XBoxOneMapping.getMapping(event) == XBoxOneMapping.A)
      {
         stepTeleopManager.requestSteppingState();
         if (stepTeleopManager.isWalking())
            stepTeleopManager.requestStanding();
      }

      if (XBoxOneMapping.getMapping(event) == XBoxOneMapping.X)
      {
         stepTeleopManager.requestXGait();
      }
   }

   @Override
   public void processEvent(Event event)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(XBoxOneMapping.getMapping(event), (double) event.getValue());

      // Handle events that should trigger once immediately after the event is triggered.
      processStateChangeRequests(event);
   }

   public void setSnapper(PointFootSnapper snapper)
   {
      stepTeleopManager.setStepSnapper(snapper);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
