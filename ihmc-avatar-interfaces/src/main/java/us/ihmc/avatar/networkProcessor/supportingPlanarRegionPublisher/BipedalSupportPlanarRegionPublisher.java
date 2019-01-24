package us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeRos2Node;

public class BipedalSupportPlanarRegionPublisher
{
   private final RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, "supporting_planar_region_publisher");
   private final IHMCRealtimeROS2Publisher<PlanarRegionsListMessage> regionPublisher;

   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusMessage = new AtomicReference<>(null);
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationData = new AtomicReference<>(null);

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> task;

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final SideDependentList<ContactablePlaneBody> contactableFeet;

   public BipedalSupportPlanarRegionPublisher(DRCRobotModel robotModel)
   {
      String robotName = robotModel.getSimpleRobotName();
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      contactableBodiesFactory.setFootContactPoints(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      contactableFeet = new SideDependentList<>(contactableBodiesFactory.createFootContactablePlaneBodies());

      double supportRegionScaleFactor = 2.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         contactableFeet.get(robotSide).getContactPoints2d().forEach(point -> point.scale(supportRegionScaleFactor));
      }

      ROS2Tools.createCallbackSubscription(ros2Node, CapturabilityBasedStatus.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           (NewMessageListener<CapturabilityBasedStatus>) subscriber -> latestCapturabilityBasedStatusMessage.set(subscriber.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           (NewMessageListener<RobotConfigurationData>) subscriber -> latestRobotConfigurationData.set(subscriber.takeNextData()));
      regionPublisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class,
                                                  REACommunicationProperties.subscriberCustomRegionsTopicNameGenerator);
   }

   public void start()
   {
      ros2Node.spin();
      task = executorService.scheduleWithFixedDelay(this::run, 0, 1, TimeUnit.SECONDS);
   }

   private void run()
   {
      CapturabilityBasedStatus capturabilityBasedStatus = latestCapturabilityBasedStatusMessage.get();
      if (capturabilityBasedStatus == null)
         return;

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationData.get();
      if (robotConfigurationData == null)
         return;

      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, fullRobotModel.getRootJoint(),
                                                                      FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel));

      fullRobotModel.getElevator().updateFramesRecursively();
      referenceFrames.updateFrames();

      SideDependentList<Boolean> isInSupport = new SideDependentList<Boolean>(!capturabilityBasedStatus.getLeftFootSupportPolygon2d().isEmpty(),
                                                                              !capturabilityBasedStatus.getRightFootSupportPolygon2d().isEmpty());

      List<PlanarRegion> supportRegions = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isInSupport.get(robotSide))
            continue;

         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         List<FramePoint2D> contactPoints = contactableFoot.getContactPoints2d();
         RigidBodyTransform transformToWorld = contactableFoot.getSoleFrame().getTransformToWorldFrame();

         PlanarRegion supportRegion = new PlanarRegion(transformToWorld, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactPoints)));
         supportRegion.setRegionId(robotSide.ordinal());
         supportRegions.add(supportRegion);
      }

      regionPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(supportRegions)));
   }

   public void stop()
   {
      task.cancel(false);
   }

   public void destroy()
   {
      stop();
      executorService.shutdownNow();
      ros2Node.destroy();
   }
}
