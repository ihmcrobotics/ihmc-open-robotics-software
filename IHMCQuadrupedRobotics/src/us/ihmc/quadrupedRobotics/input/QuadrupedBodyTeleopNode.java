package us.ihmc.quadrupedRobotics.input;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.input.mode.QuadrupedTeleopMode;
import us.ihmc.quadrupedRobotics.input.mode.QuadrupedTestTeleopMode;
import us.ihmc.quadrupedRobotics.input.mode.QuadrupedXGaitTeleopMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.subscribers.RobotDataReceiver;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

public class QuadrupedBodyTeleopNode implements InputEventCallback
{
   /**
    * Period at which to send control packets.
    */
   private static final double DT = 0.01;

   private final PollingInputDevice device;
   private final Map<InputChannel, Double> channels = Collections.synchronizedMap(new EnumMap<InputChannel, Double>(InputChannel.class));
   private final ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();

   private final YoVariableServer server;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PacketCommunicator packetCommunicator;
   private final RobotDataReceiver robotDataReceiver;
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();

   private final QuadrupedXGaitTeleopMode xGaitTeleopMode;
   private final QuadrupedTestTeleopMode testTeleopMode;
   private QuadrupedTeleopMode activeTeleopMode;

   public QuadrupedBodyTeleopNode(String host, NetClassList netClassList, PollingInputDevice device, SDFFullQuadrupedRobotModel fullRobotModel,
         QuadrupedPhysicalProperties physicalProperties) throws IOException
   {
      this.device = device;

      this.server = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName()), null, LogSettings.BEHAVIOR, DT);
      this.server.setMainRegistry(registry, fullRobotModel, new YoGraphicsListRegistry());
      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, NetworkPorts.CONTROLLER_PORT, netClassList);
      this.robotDataReceiver = new RobotDataReceiver(fullRobotModel, null);
      this.packetCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      this.taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(fullRobotModel, referenceFrames, registry);
      this.xGaitTeleopMode = new QuadrupedXGaitTeleopMode(packetCommunicator, referenceFrames);
      this.testTeleopMode = new QuadrupedTestTeleopMode(packetCommunicator, referenceFrames);

      // Set the default teleop mode.
      this.activeTeleopMode = xGaitTeleopMode;

      // Initialize all channels to zero.
      for (InputChannel channel : InputChannel.values())
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

      // Poll indefinitely.
      device.registerCallback(this);
      device.poll();
   }

   public void update()
   {
      robotDataReceiver.updateRobotModel();
      taskSpaceEstimator.compute(taskSpaceEstimates);

      activeTeleopMode.update(Collections.unmodifiableMap(channels), taskSpaceEstimates);

      server.update(robotDataReceiver.getSimTimestamp());
   }

   @Override
   public void onInputEvent(InputEvent e)
   {
      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(e.getChannel(), e.getValue());

      // Handle events that should trigger once immediately after the event is triggered.
      switch (e.getChannel())
      {
      case VIEW_BUTTON:
         activeTeleopMode.onExit();
         activeTeleopMode = testTeleopMode;
         activeTeleopMode.onEntry();
         break;
      case MENU_BUTTON:
         activeTeleopMode.onExit();
         activeTeleopMode = xGaitTeleopMode;
         activeTeleopMode.onEntry();
         break;
      default:
         // Pass any non-mode-switching events down to the active mode.
         activeTeleopMode.onInputEvent(Collections.unmodifiableMap(channels), taskSpaceEstimates, e);
      }
   }
}
