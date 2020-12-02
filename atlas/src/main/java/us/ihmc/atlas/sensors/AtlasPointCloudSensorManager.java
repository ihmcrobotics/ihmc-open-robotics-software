package us.ihmc.atlas.sensors;

import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher.StereoVisionWorldTransformCalculator;
import us.ihmc.avatar.networkProcessor.trackingCameraPublisher.TrackingCameraBridge;
import us.ihmc.avatar.networkProcessor.trackingCameraPublisher.TrackingCameraBridge.SensorFrameInitializationTransformer;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosMainNode;

public class AtlasPointCloudSensorManager
{
   private final StereoVisionPointCloudPublisher realsenseDepthPointCloudPublisher;
   private final TrackingCameraBridge trackingCameraPublisher;

   private static final String depthTopicNameToSubscribe = AtlasSensorInformation.depthCameraTopic;
   private static final String trackingTopicNameToSubscribe = AtlasSensorInformation.trackingCameraTopic;

   private final RigidBodyTransform latestTrackingSensorPose = new RigidBodyTransform();

   public AtlasPointCloudSensorManager(FullHumanoidRobotModelFactory modelFactory,
                                       ROS2NodeInterface ros2Node,
                                       RobotROSClockCalculator rosClockCalculator,
                                       boolean useTrackingData)
   {
      LogTools.info("Creating sensor bridges...");
      realsenseDepthPointCloudPublisher = new StereoVisionPointCloudPublisher(modelFactory, ros2Node, ROS2Tools.D435_POINT_CLOUD);
      LogTools.info("Using ROS clock calculator {}", rosClockCalculator.getClass());
      realsenseDepthPointCloudPublisher.setROSClockCalculator(rosClockCalculator);

      trackingCameraPublisher = new TrackingCameraBridge(modelFactory, ros2Node);
      trackingCameraPublisher.setROSClockCalculator(rosClockCalculator);
      trackingCameraPublisher.setCustomInitializationTransformer(createCustomTrackingCameraWorldTransformCalculator());

      if (useTrackingData)
         realsenseDepthPointCloudPublisher.setCustomStereoVisionTransformer(createDepthPointCloudWorldTransformCalculatorFromTrackingData());
      else
         realsenseDepthPointCloudPublisher.setCustomStereoVisionTransformer(createDepthPointCloudWorldTransformCalculator());
   }

   public void receiveDataFromROS1(RosMainNode rosMainNode)
   {
      realsenseDepthPointCloudPublisher.receiveStereoPointCloudFromROS1(depthTopicNameToSubscribe, rosMainNode);
      trackingCameraPublisher.receiveTrackingCameraDataFromROS1(trackingTopicNameToSubscribe, rosMainNode);
   }

   public void start()
   {
      realsenseDepthPointCloudPublisher.start();
      trackingCameraPublisher.start();
   }

   public void shutdown()
   {
      realsenseDepthPointCloudPublisher.shutdown();
      trackingCameraPublisher.shutdown();
   }
   
   public void setCollisionBoxProvider(CollisionBoxProvider collisionBoxProvider)
   {
      realsenseDepthPointCloudPublisher.setSelfCollisionFilter(collisionBoxProvider);
   }

   private StereoVisionWorldTransformCalculator createDepthPointCloudWorldTransformCalculator()
   {
      return new StereoVisionWorldTransformCalculator()
      {
         private final RigidBodyTransform transformFromPelvisToRealSense = AtlasSensorInformation.transformPelvisToDepthCamera;

         @Override
         public void computeTransformToWorld(FullRobotModel fullRobotModel, RigidBodyTransform transformToWorldToPack, Pose3DBasics sensorPoseToPack)
         {
            ReferenceFrame pelvisFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
            pelvisFrame.getTransformToDesiredFrame(transformToWorldToPack, ReferenceFrame.getWorldFrame());
            transformToWorldToPack.multiply(transformFromPelvisToRealSense);
            sensorPoseToPack.set(transformToWorldToPack);
         }
      };
   }

   private StereoVisionWorldTransformCalculator createDepthPointCloudWorldTransformCalculatorFromTrackingData()
   {
      return new StereoVisionWorldTransformCalculator()
      {
         private final RigidBodyTransform transformFromPelvisToRealSense = AtlasSensorInformation.transformTrackingCameraToDepthCamera;

         @Override
         public void computeTransformToWorld(FullRobotModel fullRobotModel, RigidBodyTransform transformToWorldToPack, Pose3DBasics sensorPoseToPack)
         {
            StampedPosePacket newTrackingSensorPose = trackingCameraPublisher.pollNewData();
            if (newTrackingSensorPose != null)
            {
               Pose3D sensorPose = newTrackingSensorPose.getPose();
               latestTrackingSensorPose.set(sensorPose.getOrientation(), sensorPose.getPosition());
            }

            transformToWorldToPack.set(latestTrackingSensorPose);
            transformToWorldToPack.multiply(transformFromPelvisToRealSense);
            sensorPoseToPack.set(transformToWorldToPack);
         }
      };
   }

   /**
    * Tracking Camera (T265) is able to compensate flat ground itself.
    * See the documentation of BMI055, Session 5.6.7.
    * So the initial transformation matrix will be snap into Zup at `TrackingCameraPublisher`.
    */
   private SensorFrameInitializationTransformer createCustomTrackingCameraWorldTransformCalculator()
   {
      return new SensorFrameInitializationTransformer()
      {
         private final RigidBodyTransform transformFromPelvisToRealSense = AtlasSensorInformation.transformPelvisToTrackingCamera;

         @Override
         public void computeTransformToWorld(FullRobotModel fullRobotModel, RigidBodyTransform transformToWorldToPack)
         {
            ReferenceFrame pelvisFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
            pelvisFrame.getTransformToDesiredFrame(transformToWorldToPack, ReferenceFrame.getWorldFrame());
            transformToWorldToPack.multiply(transformFromPelvisToRealSense);
         }
      };
   }
}
