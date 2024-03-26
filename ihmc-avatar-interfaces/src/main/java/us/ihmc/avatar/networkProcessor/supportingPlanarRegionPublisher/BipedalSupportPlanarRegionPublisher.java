package us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class BipedalSupportPlanarRegionPublisher implements CloseableAndDisposable
{
   public static final double defaultScaleFactor = 2.0;

   private final boolean manageROS2Node;
   private final RealtimeROS2Node ros2Node;
   private final IHMCRealtimeROS2Publisher<PlanarRegionsListMessage> regionPublisher;

   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusMessage = new AtomicReference<>(null);
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationData = new AtomicReference<>(null);
   private final AtomicReference<BipedalSupportPlanarRegionParametersMessage> latestParametersMessage = new AtomicReference<>(null);

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> task;

   private final BipedalSupportPlanarRegionCalculator bipedalSupportPlanarRegionCalculator;

   public BipedalSupportPlanarRegionPublisher(DRCRobotModel robotModel, RealtimeROS2Node realtimeROS2Node)
   {
      this(robotModel, realtimeROS2Node, null);
   }

   public BipedalSupportPlanarRegionPublisher(DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      this(robotModel, null, pubSubImplementation);
   }

   private BipedalSupportPlanarRegionPublisher(DRCRobotModel robotModel, RealtimeROS2Node realtimeROS2Node, PubSubImplementation pubSubImplementation)
   {
      String robotName = robotModel.getSimpleRobotName();
      bipedalSupportPlanarRegionCalculator = new BipedalSupportPlanarRegionCalculator(robotModel);

      manageROS2Node = realtimeROS2Node == null;
      if (realtimeROS2Node == null)
         realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, "supporting_planar_region_publisher");
      ros2Node = realtimeROS2Node;

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    CapturabilityBasedStatus.class,
                                                    ROS2Tools.getControllerOutputTopic(robotName),
                                                    subscriber -> latestCapturabilityBasedStatusMessage.set(subscriber.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    ROS2Tools.getControllerOutputTopic(robotName),
                                                    subscriber -> latestRobotConfigurationData.set(subscriber.takeNextData()));
      regionPublisher = ROS2Tools.createPublisher(ros2Node, PerceptionAPI.BIPEDAL_SUPPORT_REGIONS);
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           getTopic(robotName),
                                           s -> latestParametersMessage.set(s.takeNextData()));

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
