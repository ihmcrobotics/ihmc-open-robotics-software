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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   /**
    * If true, the support polygon's convex hull is published as a single planar region if the robot is in double support and the feet planes are close.
    * Otherwise, the polygons are published separately
    */
   private static final boolean useConvexHullIfPossible = true;
   private static final double supportRegionScaleFactor = 2.0;

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

      if (useConvexHullIfPossible && feetAreInSamePlane(isInSupport))
      {
         ContactablePlaneBody leftContactableFoot = contactableFeet.get(RobotSide.LEFT);
         ContactablePlaneBody rightContactableFoot = contactableFeet.get(RobotSide.RIGHT);

         ReferenceFrame leftSoleFrame = leftContactableFoot.getSoleFrame();
         RigidBodyTransform leftFootTransformToWorld = leftSoleFrame.getTransformToWorldFrame();

         List<FramePoint2D> leftFootContactPoints = leftContactableFoot.getContactPoints2d();
         List<FramePoint2D> rightFootContactPoints = rightContactableFoot.getContactPoints2d();
         List<FramePoint2D> allContactPoints = new ArrayList<>(leftFootContactPoints);
         allContactPoints.addAll(rightFootContactPoints);
         allContactPoints.forEach(point -> point.changeFrame(leftSoleFrame));

         PlanarRegion convexHullRegion = new PlanarRegion(leftFootTransformToWorld, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(allContactPoints)));
         convexHullRegion.setRegionId(0);
         supportRegions.add(convexHullRegion);
      }
      else
      {
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
      }

      regionPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(supportRegions)));
   }

   private boolean feetAreInSamePlane(SideDependentList<Boolean> isInSupport)
   {
      boolean inDoubleSupport = isInSupport.get(RobotSide.LEFT) && isInSupport.get(RobotSide.RIGHT);
      ReferenceFrame leftSoleFrame = contactableFeet.get(RobotSide.LEFT).getSoleFrame();
      ReferenceFrame rightSoleFrame = contactableFeet.get(RobotSide.RIGHT).getSoleFrame();
      RigidBodyTransform relativeSoleTransform = leftSoleFrame.getTransformToDesiredFrame(rightSoleFrame);

      double rotationEpsilon = Math.toRadians(3.0);
      double translationEpsilon = 0.02;
      return inDoubleSupport && relativeSoleTransform.getRotationMatrix().getPitch() < rotationEpsilon && relativeSoleTransform.getRotationMatrix().getRoll() < rotationEpsilon
            && relativeSoleTransform.getTranslationZ() < translationEpsilon;
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
