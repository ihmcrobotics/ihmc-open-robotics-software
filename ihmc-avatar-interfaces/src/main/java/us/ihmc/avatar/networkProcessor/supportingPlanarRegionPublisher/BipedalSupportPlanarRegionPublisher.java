package us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher;

import controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class BipedalSupportPlanarRegionPublisher implements CloseableAndDisposable
{
   public static final double defaultScaleFactor = 2.0;

   private final boolean manageROS2Node;
   private final RealtimeROS2Node ros2Node;
   private final ROS2Publisher<PlanarRegionsListMessage> regionPublisher;

   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusMessage = new AtomicReference<>(null);
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationData = new AtomicReference<>(null);
   private final AtomicReference<BipedalSupportPlanarRegionParametersMessage> latestParametersMessage = new AtomicReference<>(null);

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> task;

   private final BipedalSupportPlanarRegionCalculator bipedalSupportPlanarRegionCalculator;

   public BipedalSupportPlanarRegionPublisher(DRCRobotModel robotModel)
   {
      this(robotModel, null);
   }

   public BipedalSupportPlanarRegionPublisher(DRCRobotModel robotModel, RealtimeROS2Node realtimeROS2Node)
   {
      String robotName = robotModel.getSimpleRobotName();
      bipedalSupportPlanarRegionCalculator = new BipedalSupportPlanarRegionCalculator(robotModel);

      manageROS2Node = realtimeROS2Node == null;
      if (realtimeROS2Node == null)
         realtimeROS2Node = new ROS2NodeBuilder().buildRealtime("supporting_planar_region_publisher");
      ros2Node = realtimeROS2Node;

      ros2Node.createSubscription(HumanoidControllerAPI.getTopic(CapturabilityBasedStatus.class, robotName),
                                  subscriber -> latestCapturabilityBasedStatusMessage.set(subscriber.takeNextData()));
      ros2Node.createSubscription(StateEstimatorAPI.getRobotConfigurationDataTopic(robotName),
                                  subscriber -> latestRobotConfigurationData.set(subscriber.takeNextData()));
      regionPublisher = ros2Node.createPublisher(PerceptionAPI.BIPEDAL_SUPPORT_REGIONS);
      ros2Node.createSubscription(getTopic(robotName), subscriber -> latestParametersMessage.set(subscriber.takeNextData()));

      BipedalSupportPlanarRegionParametersMessage defaultParameters = new BipedalSupportPlanarRegionParametersMessage();
      defaultParameters.setEnable(true);
      defaultParameters.setSupportRegionScaleFactor(defaultScaleFactor);
      latestParametersMessage.set(defaultParameters);

      bipedalSupportPlanarRegionCalculator.initializeEmptyRegions();
   }

   public void start()
   {
      if (manageROS2Node)
         ros2Node.spin();
      task = executorService.scheduleWithFixedDelay(this::run, 0, 1, TimeUnit.SECONDS);
   }

   private void run()
   {
      BipedalSupportPlanarRegionParametersMessage parameters = latestParametersMessage.get();
      if (!parameters.getEnable() || parameters.getSupportRegionScaleFactor() <= 0.0)
      {
         bipedalSupportPlanarRegionCalculator.initializeEmptyRegions();
         publishRegions();
         return;
      }

      CapturabilityBasedStatus capturabilityBasedStatus = latestCapturabilityBasedStatusMessage.get();
      if (capturabilityBasedStatus == null)
      {
         return;
      }

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationData.get();
      if (robotConfigurationData == null)
      {
         return;
      }

      bipedalSupportPlanarRegionCalculator.calculateSupportRegions(parameters.getSupportRegionScaleFactor(), capturabilityBasedStatus, robotConfigurationData);

      publishRegions();
   }

   private void publishRegions()
   {
      regionPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(bipedalSupportPlanarRegionCalculator.getSupportRegionsAsList()));
   }

   public void stop()
   {
      task.cancel(false);
   }

   public void destroy()
   {
      stop();
      executorService.shutdownNow();
      if (manageROS2Node)
         ros2Node.destroy();
   }

   @Override
   public void closeAndDispose()
   {
      destroy();
   }

   public static ROS2Topic<BipedalSupportPlanarRegionParametersMessage> getTopic(String robotName)
   {
      return PerceptionAPI.getBipedalSupportRegionParametersTopic(robotName);
   }
}
