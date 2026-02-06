package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.depthData.CollisionShapeTester;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class DelayFixedPlanarRegionsSubscription
{
   public static final double INITIAL_DELAY_OFFSET = 0.07; // TODO: Put in a stored property set

   private final ResettableExceptionHandlingExecutorService executorService;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final HumanoidReferenceFrames referenceFrames;
   private final ROS2Node ros2Node;
   private final DRCRobotModel robotModel;
   private final String topic;
   private final Consumer<Pair<Long, PlanarRegionsList>> callback;
   private final MutableDouble delayOffset = new MutableDouble(INITIAL_DELAY_OFFSET);
   private final FullHumanoidRobotModel fullRobotModel;
   private final RobotROSClockCalculator rosClockCalculator;
   private ROS2Subscription<?> robotConfigurationDataSubscriber;
   private boolean posePublisherEnabled = false;

   private CollisionBoxProvider collisionBoxProvider;
   private CollidingScanRegionFilter collisionFilter;

   private boolean enabled = false;
   private double delay = 0.0;

   public DelayFixedPlanarRegionsSubscription(ROS2Node ros2Node,
                                              DRCRobotModel robotModel,
                                              String topic,
                                              Consumer<Pair<Long, PlanarRegionsList>> callback)
   {
      this.ros2Node = ros2Node;
      this.robotModel = robotModel;
      this.topic = topic;
      this.callback = callback;

      rosClockCalculator = robotModel.getROSClockCalculator();
      ros2Node.createSubscription2(StateEstimatorAPI.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()),
                                   rosClockCalculator::receivedRobotConfigurationData);

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor("ROS1PlanarRegionsSubscriber", daemon, queueSize);

      fullRobotModel = robotModel.createFullRobotModel();
      robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

      referenceFrames = new HumanoidReferenceFrames(fullRobotModel, robotModel.getSensorInformation());

      collisionBoxProvider = robotModel.getCollisionBoxProvider();
      CollisionShapeTester shapeTester = new CollisionShapeTester();
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
      }
      collisionFilter = new CollidingScanRegionFilter(shapeTester);
   }


   private void acceptRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      // LogTools.info("Recieved robot configuration data w/ timestamp: {}", data.getMonotonicTime());
      robotConfigurationDataBuffer.update(robotConfigurationData);
   }

//   private void acceptRawGPUPlanarRegionsList(RawGPUPlanarRegionList rawGPUPlanarRegionList)
//   {
//      if (enabled)
//      {
//         executorService.clearQueueAndExecute(() ->
//         {
//            long timestamp = rawGPUPlanarRegionList.getHeader().getStamp().totalNsecs();
//            //      LogTools.info("rawGPU timestamp: {}", timestamp);
//            double seconds = delayOffset.getValue();
//            //      LogTools.info("Latest delay: {}", seconds);
//            timestamp -= Conversions.secondsToNanoseconds(seconds);
//
//
////            if (!rosClockCalculator.offsetIsDetermined())
////            {
////               delay = Double.NaN;
////               return;
////            }
////
////            long controllerTime = rosClockCalculator.computeRobotMonotonicTime(timestamp);
////            if (controllerTime == -1L)
////            {
////               delay = Double.NaN;
////               return;
////            }
//
//            long newestTimestamp = robotConfigurationDataBuffer.getNewestTimestamp();
//            if (newestTimestamp == -1L)
//            {
//               delay = Double.NaN;
//               return;
//            }
//
//            long controllerTime = newestTimestamp - Conversions.millisecondsToNanoseconds(250);
//
//            boolean waitIfNecessary = false; // dangerous if true! need a timeout
//            long selectedTimestamp = robotConfigurationDataBuffer.updateFullRobotModel(waitIfNecessary, controllerTime, fullRobotModel, null);
//            if (selectedTimestamp != -1L)
//            {
////               long currentTimeInWall = ros1Node.getCurrentTime().totalNsecs();
////               long selectedTimeInWall = selectedTimestamp - rosClockCalculator.getCurrentTimestampOffset();
////               delay = Conversions.nanosecondsToSeconds(currentTimeInWall - selectedTimeInWall);
////
////               try
////               {
////                  referenceFrames.updateFrames();
////               }
////               catch (NotARotationMatrixException e)
////               {
////                  LogTools.error(e.getMessage());
////               }
////
////               PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
////               try
////               {
////                  planarRegionsList.applyTransform(MapsenseTools.getTransformFromCameraToWorld());
////                  planarRegionsList.applyTransform(referenceFrames.getSteppingCameraFrame().getTransformToWorldFrame());
////
////                  collisionFilter.update();
////                  gpuPlanarRegionUpdater.filterCollidingPlanarRegions(planarRegionsList, collisionFilter);
////
////                  if(posePublisherEnabled)
////                  {
////                     RigidBodyTransform transform = referenceFrames.getSteppingCameraFrame().getTransformToWorldFrame();
////                     sensorPosePublisher.publish("world",
////                                                 (Vector3D) transform.getTranslation(),
////                                                 new Quaternion(transform.getRotation()),
////                                                 new Time(currentTimeInWall));
////                     transform = referenceFrames.getMidFeetUnderPelvisFrame().getTransformToWorldFrame();
////                     pelvisPosePublisher.publish("world",
////                                                 (Vector3D) transform.getTranslation(),
////                                                 new Quaternion(transform.getRotation()),
////                                                 new Time(currentTimeInWall));
////                  }
////               }
////               catch (NotARotationMatrixException e)
////               {
////                  LogTools.error(e.getMessage());
////               }
////               callback.accept(Pair.of(currentTimeInWall, planarRegionsList));
//            }
//         });
//      }
//   }

   public void destroy()
   {
      executorService.destroy();
   }

   public void setEnabled(boolean enabled)
   {
      if (this.enabled != enabled)
      {
         if (enabled)
         {
            robotConfigurationDataSubscriber = ros2Node.createSubscription2(StateEstimatorAPI.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()),
                                                                            this::acceptRobotConfigurationData);
         }
         else
         {
            executorService.interruptAndReset();
            robotConfigurationDataSubscriber.remove();
            robotConfigurationDataSubscriber = null;
         }
      }

      this.enabled = enabled;
   }

   public double getDelay()
   {
      return delay;
   }

   public void setPosePublisherEnabled(boolean posePublisherEnabled)
   {
      this.posePublisherEnabled = posePublisherEnabled;
   }
}
