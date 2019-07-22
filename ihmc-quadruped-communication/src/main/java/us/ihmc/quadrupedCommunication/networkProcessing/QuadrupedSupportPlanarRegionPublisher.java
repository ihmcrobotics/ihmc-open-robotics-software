package us.ihmc.quadrupedCommunication.networkProcessing;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import org.ejml.data.DenseMatrix64F;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.QuadrupedSupportPlanarRegionParametersMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeRos2Node;

public class QuadrupedSupportPlanarRegionPublisher
{
   private static final double defaultInsideRegionSize = 0.3;
   private static final double defaultOutsideRegionSize = 0.1;
   private static final double heightMergeThreshold = 0.02;

   private static final int FRONT_LEFT_FOOT_INDEX = 0;
   private static final int FRONT_RIGHT_FOOT_INDEX = 1;
   private static final int HIND_LEFT_FOOT_INDEX = 2;
   private static final int HIND_RIGHT_FOOT_INDEX = 3;
   private static final int HIND_CONVEX_HULL_INDEX = 4;
   private static final int FRONT_CONVEX_HULL_INDEX = 5;

   private static final int numberOfRegions = 6;

   private final RealtimeRos2Node ros2Node;
   private final IHMCRealtimeROS2Publisher<PlanarRegionsListMessage> regionPublisher;

   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationData = new AtomicReference<>(null);
   private final AtomicReference<QuadrupedSupportPlanarRegionParametersMessage> latestParametersMessage = new AtomicReference<>(null);

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> task;

   private final FullQuadrupedRobotModel fullRobotModel;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrantDependentList<MovingReferenceFrame> soleZUpFrames;
   private final List<PlanarRegion> supportRegions = new ArrayList<>();

   public QuadrupedSupportPlanarRegionPublisher(FullQuadrupedRobotModelFactory robotModel, QuadrantDependentList<ArrayList<Point2D>> groundContactPoints,
                                                PubSubImplementation pubSubImplementation)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      oneDoFJoints = fullRobotModel.getOneDoFJoints();
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel);
      String robotName = robotModel.getRobotDescription().getName();

      soleZUpFrames = referenceFrames.getSoleZUpFrames();

      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "supporting_planar_region_publisher");

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           RobotConfigurationData.class,
                                           QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           (NewMessageListener<RobotConfigurationData>) subscriber -> latestRobotConfigurationData.set(subscriber.takeNextData()));
      regionPublisher = ROS2Tools.createPublisher(ros2Node,
                                                  PlanarRegionsListMessage.class,
                                                  REACommunicationProperties.subscriberCustomRegionsTopicNameGenerator);
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           QuadrupedSupportPlanarRegionParametersMessage.class,
                                           ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.QUADRUPED_SUPPORT_REGION_PUBLISHER, ROS2TopicQualifier.INPUT),
                                           s -> latestParametersMessage.set(s.takeNextData()));

      QuadrupedSupportPlanarRegionParametersMessage defaultParameters = new QuadrupedSupportPlanarRegionParametersMessage();
      defaultParameters.setEnable(true);
      defaultParameters.setInsideSupportRegionSize(defaultInsideRegionSize);
      defaultParameters.setOutsideSupportRegionSize(defaultOutsideRegionSize);
      latestParametersMessage.set(defaultParameters);

      for (int i = 0; i < numberOfRegions; i++)
      {
         supportRegions.add(new PlanarRegion());
      }
   }

   public void start()
   {
      ros2Node.spin();
      task = executorService.scheduleWithFixedDelay(this::run, 0, 1, TimeUnit.SECONDS);
   }

   public void close()
   {
      ros2Node.destroy();
      task.cancel(true);
   }

   private void run()
   {
      QuadrupedSupportPlanarRegionParametersMessage parameters = latestParametersMessage.get();
      if (!parameters.getEnable() || parameters.getInsideSupportRegionSize() <= 0.0 || parameters.getOutsideSupportRegionSize() <= 0.0)
      {
         supportRegions.set(FRONT_LEFT_FOOT_INDEX, new PlanarRegion());
         supportRegions.set(FRONT_RIGHT_FOOT_INDEX, new PlanarRegion());
         supportRegions.set(HIND_LEFT_FOOT_INDEX, new PlanarRegion());
         supportRegions.set(HIND_RIGHT_FOOT_INDEX, new PlanarRegion());
         supportRegions.set(HIND_CONVEX_HULL_INDEX, new PlanarRegion());
         supportRegions.set(FRONT_CONVEX_HULL_INDEX, new PlanarRegion());

         publishRegions();
         return;
      }

      double insideSupportRegionSize = parameters.getInsideSupportRegionSize();
      double outsideSupportRegionSize = parameters.getOutsideSupportRegionSize();

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationData.get();
      if (robotConfigurationData == null)
         return;

      setRobotStateFromRobotConfigurationData(robotConfigurationData, fullRobotModel.getRootJoint(), oneDoFJoints);

      referenceFrames.updateFrames();

      QuadrantDependentList<Boolean> isInSupport = new QuadrantDependentList<>(true, true, true, true);

      for (RobotEnd robotEnd : RobotEnd.values)
      {
         if (feetAreInSamePlane(robotEnd, isInSupport))
         {
            ReferenceFrame leftSoleFrame = soleZUpFrames.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.LEFT));

            List<FramePoint2D> allContactPoints = new ArrayList<>();

            for (RobotSide robotSide : RobotSide.values)
            {
               RobotQuadrant robotQuadrant = RobotQuadrant.getQuadrant(robotEnd, robotSide);
               allContactPoints.addAll(createConvexPolygonPoints(robotQuadrant,
                                                                 soleZUpFrames.get(robotQuadrant),
                                                                 insideSupportRegionSize,
                                                                 outsideSupportRegionSize));
            }

            allContactPoints.forEach(p -> p.changeFrameAndProjectToXYPlane(leftSoleFrame));

            for (RobotSide robotSide : RobotSide.values)
            {
               RobotQuadrant robotQuadrant = RobotQuadrant.getQuadrant(robotEnd, robotSide);
               supportRegions.set(robotQuadrant.ordinal(), new PlanarRegion());
            }

            supportRegions.set(4 + robotEnd.ordinal(),
                               new PlanarRegion(leftSoleFrame.getTransformToWorldFrame(),
                                                new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(allContactPoints))));
         }
         else
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               RobotQuadrant robotQuadrant = RobotQuadrant.getQuadrant(robotEnd, robotSide);
               if (isInSupport.get(robotQuadrant))
               {
                  ReferenceFrame soleZUpFrame = soleZUpFrames.get(robotQuadrant);
                  List<FramePoint2D> contactPoints = createConvexPolygonPoints(robotQuadrant, soleZUpFrame, insideSupportRegionSize, outsideSupportRegionSize);
                  RigidBodyTransform transformToWorld = soleZUpFrame.getTransformToWorldFrame();
                  supportRegions.set(robotQuadrant.ordinal(),
                                     new PlanarRegion(transformToWorld, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactPoints))));
               }
               else
               {
                  supportRegions.set(robotQuadrant.ordinal(), new PlanarRegion());
               }
            }

            supportRegions.set(4 + robotEnd.ordinal(), new PlanarRegion());
         }
      }

      publishRegions();
   }

   private void publishRegions()
   {
      for (int i = 0; i < numberOfRegions; i++)
      {
         supportRegions.get(i).setRegionId(i);
      }

      regionPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(supportRegions)));
   }

   private boolean feetAreInSamePlane(RobotEnd robotEnd, QuadrantDependentList<Boolean> isInSupport)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isInSupport.get(RobotQuadrant.getQuadrant(robotEnd, robotSide)))
            return false;
      }

      ReferenceFrame leftSoleFrame = soleZUpFrames.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.LEFT));
      ReferenceFrame rightSoleFrame = soleZUpFrames.get(RobotQuadrant.getQuadrant(robotEnd, RobotSide.RIGHT));

      RigidBodyTransform leftToRight = leftSoleFrame.getTransformToDesiredFrame(rightSoleFrame);

      return Math.abs(leftToRight.getTranslationZ()) < heightMergeThreshold;
   }

   private static List<FramePoint2D> createConvexPolygonPoints(RobotQuadrant robotQuadrant, ReferenceFrame referenceFrame, double insideSize,
                                                               double outsideSize)
   {
      List<FramePoint2D> points = new ArrayList<>();
      double forwardBound;
      double backwardBound;
      double leftBound;
      double rightBound;

      if (robotQuadrant.isQuadrantInFront())
      {
         forwardBound = insideSize;
         backwardBound = -outsideSize;
         if (robotQuadrant.isQuadrantOnLeftSide())
         {
            leftBound = insideSize;
            rightBound = -outsideSize;
         }
         else
         {
            leftBound = outsideSize;
            rightBound = -insideSize;
         }
      }
      else
      {
         forwardBound = insideSize;
         backwardBound = -outsideSize;

         if (robotQuadrant.isQuadrantOnLeftSide())
         {
            leftBound = outsideSize;
            rightBound = -insideSize;
         }
         else
         {
            leftBound = insideSize;
            rightBound = -outsideSize;
         }
      }

      points.add(new FramePoint2D(referenceFrame, forwardBound, leftBound));
      points.add(new FramePoint2D(referenceFrame, forwardBound, rightBound));
      points.add(new FramePoint2D(referenceFrame, backwardBound, leftBound));
      points.add(new FramePoint2D(referenceFrame, backwardBound, rightBound));

      return points;
   }

   public static void setRobotStateFromRobotConfigurationData(RobotConfigurationData robotConfigurationData, FloatingJointBasics desiredRootJoint,
                                                              OneDoFJointBasics[] oneDoFJoints)
   {
      TFloatArrayList newJointAngles = robotConfigurationData.getJointAngles();

      for (int i = 0; i < newJointAngles.size(); i++)
      {
         oneDoFJoints[i].setQ(newJointAngles.get(i));
         oneDoFJoints[i].setQd(0.0);
      }

      if (desiredRootJoint != null)
      {
         Vector3D translation = robotConfigurationData.getRootTranslation();
         desiredRootJoint.getJointPose().setPosition(translation.getX(), translation.getY(), translation.getZ());
         Quaternion orientation = robotConfigurationData.getRootOrientation();
         desiredRootJoint.getJointPose().getOrientation().setQuaternion(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
         desiredRootJoint.setJointVelocity(0, new DenseMatrix64F(6, 1));

         desiredRootJoint.getPredecessor().updateFramesRecursively();
      }
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
