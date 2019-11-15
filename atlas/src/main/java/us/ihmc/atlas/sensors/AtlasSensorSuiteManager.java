package us.ihmc.atlas.sensors;

import java.io.IOException;
import java.net.URI;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.RobotPhysicalProperties;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.lidarScanPublisher.LidarScanPublisher;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher.StereoVisionWorldTransformCalculator;
import us.ihmc.avatar.networkProcessor.trackingCameraPublisher.TrackingCameraPublisher;
import us.ihmc.avatar.networkProcessor.trackingCameraPublisher.TrackingCameraPublisher.SensorFrameInitializationTransformer;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.avatar.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.ihmcPerception.camera.FisheyeCameraReceiver;
import us.ihmc.ihmcPerception.camera.SCSCameraDataReceiver;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasSensorSuiteManager implements DRCSensorSuiteManager
{
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_atlas_sensor_suite_node");

   private final LidarScanPublisher lidarScanPublisher;
   private final StereoVisionPointCloudPublisher multisenseStereoVisionPointCloudPublisher;
   private final StereoVisionPointCloudPublisher realsenseDepthPointCloudPublisher;
   private final TrackingCameraPublisher trackingCameraPublisher;

   private static final boolean ENABLE_STEREO_PUBLISHER = false;
   private static final boolean ENABLE_DEPTH_PUBLISHER = true;
   private static final boolean ENABLE_TRACKING_PUBLISHER = true;
   private static final boolean USE_DEPTH_FRAME_ESTIMATED_BY_TRACKING = true;

   private static final String topicNamePrefixToPublish = ROS2Tools.IHMC_ROS_TOPIC_PREFIX;
   private static final String depthTopicNameSurfixToPublish = "_D435";
   private static final String depthTopicNameToSubscribe = AtlasSensorInformation.depthCameraTopic;
   private static final String trackingTopicNameSurfixToPublish = "_T265";
   private static final String trackingTopicNameToSubscribe = AtlasSensorInformation.trackingCameraTopic;
   private final MessageTopicNameGenerator depthCloudTopicNameGenerator;
   private final MessageTopicNameGenerator trackingCameraTopicNameGenerator;

   private final RobotROSClockCalculator rosClockCalculator;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final FullHumanoidRobotModelFactory modelFactory;

   private final String robotName;

   public AtlasSensorSuiteManager(String robotName, FullHumanoidRobotModelFactory modelFactory, CollisionBoxProvider collisionBoxProvider,
                                  RobotROSClockCalculator rosClockCalculator, HumanoidRobotSensorInformation sensorInformation, DRCRobotJointMap jointMap,
                                  RobotPhysicalProperties physicalProperties, RobotTarget targetDeployment)
   {
      this.robotName = robotName;
      this.rosClockCalculator = rosClockCalculator;
      this.sensorInformation = sensorInformation;
      this.robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      this.modelFactory = modelFactory;

      AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      String sensorName = multisenseLidarParameters.getSensorNameInSdf();
      String rcdTopicName = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName).generateTopicName(RobotConfigurationData.class);
      lidarScanPublisher = new LidarScanPublisher(sensorName, modelFactory, ros2Node, rcdTopicName);
      lidarScanPublisher.setROSClockCalculator(rosClockCalculator);
      lidarScanPublisher.setCollisionBoxProvider(collisionBoxProvider);

      multisenseStereoVisionPointCloudPublisher = new StereoVisionPointCloudPublisher(modelFactory, ros2Node, rcdTopicName);
      multisenseStereoVisionPointCloudPublisher.setROSClockCalculator(rosClockCalculator);

      depthCloudTopicNameGenerator = (Class<?> T) -> ROS2Tools.appendTypeToTopicName(topicNamePrefixToPublish, T) + depthTopicNameSurfixToPublish;
      realsenseDepthPointCloudPublisher = new StereoVisionPointCloudPublisher(modelFactory, ros2Node, rcdTopicName, depthCloudTopicNameGenerator);
      realsenseDepthPointCloudPublisher.setROSClockCalculator(rosClockCalculator);

      trackingCameraTopicNameGenerator = (Class<?> T) -> ROS2Tools.appendTypeToTopicName(topicNamePrefixToPublish, T) + trackingTopicNameSurfixToPublish;
      trackingCameraPublisher = new TrackingCameraPublisher(modelFactory, ros2Node, rcdTopicName, trackingCameraTopicNameGenerator);
      trackingCameraPublisher.setROSClockCalculator(rosClockCalculator);
      trackingCameraPublisher.setCustomInitializationTransformer(createCustomTrackingCameraWorldTransformCalculator());
      trackingCameraPublisher.setTransformToOtherSensorFrame(AtlasSensorInformation.transformTrackingCameraToDepthCamera);

      if (USE_DEPTH_FRAME_ESTIMATED_BY_TRACKING)
         realsenseDepthPointCloudPublisher.setCustomStereoVisionTransformer(trackingCameraPublisher);
      else
         realsenseDepthPointCloudPublisher.setCustomStereoVisionTransformer(createCustomDepthPointCloudWorldTransformCalculator());
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

      SCSCameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(sensorInformation.getCameraParameters(0).getRobotSide(), modelFactory,
                                                                           sensorInformation.getCameraParameters(0).getSensorNameInSdf(),
                                                                           robotConfigurationDataBuffer, scsSensorsCommunicator, ros2Node,
                                                                           rosClockCalculator::computeRobotMonotonicTime);
      cameraDataReceiver.start();

      lidarScanPublisher.receiveLidarFromSCS(scsSensorsCommunicator);
      lidarScanPublisher.setScanFrameToLidarSensorFrame();
      lidarScanPublisher.start();
   }

   @Override
   public void initializePhysicalSensors(URI rosCoreURI)
   {
      if (rosCoreURI == null)
         throw new RuntimeException(getClass().getSimpleName() + " Physical sensor requires rosURI to be set in " + NetworkParameters.defaultParameterFile);

      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, "atlas/sensorSuiteManager", true);

      AvatarRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      AvatarRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);

      lidarScanPublisher.receiveLidarFromROSAsPointCloud2WithSource(multisenseLidarParameters.getRosTopic(), rosMainNode);
      lidarScanPublisher.setScanFrameToWorldFrame();

      if (ENABLE_STEREO_PUBLISHER)
         multisenseStereoVisionPointCloudPublisher.receiveStereoPointCloudFromROS(multisenseStereoParameters.getRosTopic(), rosMainNode);
      if (ENABLE_DEPTH_PUBLISHER)
         realsenseDepthPointCloudPublisher.receiveStereoPointCloudFromROS(depthTopicNameToSubscribe, rosMainNode);
      if (ENABLE_TRACKING_PUBLISHER)
         trackingCameraPublisher.receiveTrackingCameraDataFromROS(trackingTopicNameToSubscribe, rosMainNode);

      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(modelFactory, robotConfigurationDataBuffer, rosMainNode, ros2Node,
                                                                                    rosClockCalculator, multisenseLeftEyeCameraParameters,
                                                                                    multisenseLidarParameters, multisenseStereoParameters,
                                                                                    sensorInformation.setupROSParameterSetters());

      AvatarRobotCameraParameters leftFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_LEFT_CAMERA_ID);
      AvatarRobotCameraParameters rightFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_RIGHT_CAMERA_ID);

      FisheyeCameraReceiver leftFishEyeCameraReceiver = new FisheyeCameraReceiver(modelFactory, leftFishEyeCameraParameters, robotConfigurationDataBuffer,
                                                                                  ros2Node, rosClockCalculator::computeRobotMonotonicTime, rosMainNode);
      FisheyeCameraReceiver rightFishEyeCameraReceiver = new FisheyeCameraReceiver(modelFactory, rightFishEyeCameraParameters, robotConfigurationDataBuffer,
                                                                                   ros2Node, rosClockCalculator::computeRobotMonotonicTime, rosMainNode);

      leftFishEyeCameraReceiver.start();
      rightFishEyeCameraReceiver.start();
      lidarScanPublisher.start();

      if (ENABLE_STEREO_PUBLISHER)
         multisenseStereoVisionPointCloudPublisher.start();
      if (ENABLE_DEPTH_PUBLISHER)
         realsenseDepthPointCloudPublisher.start();
      if (ENABLE_TRACKING_PUBLISHER)
         trackingCameraPublisher.start();

      rosClockCalculator.setROSMainNode(rosMainNode);

      multiSenseSensorManager.initializeParameterListeners();

      rosMainNode.execute();
      while (!rosMainNode.isStarted())
      {
         System.out.println("waiting for " + rosMainNode.getDefaultNodeName() + " to start. ");
         ThreadTools.sleep(2000);
      }
   }

   @Override
   public void connect() throws IOException
   {

   }

   private StereoVisionWorldTransformCalculator createCustomDepthPointCloudWorldTransformCalculator()
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

   /**
    * Tracking Camera (T265) is able to compensate flat ground itself.
    * See the documentation of BMI055, Session 5.6.7.
    * So the initial transformation matrix will be snap into Zup.
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
            
            RigidBodyTransform syncronizedZup = new RigidBodyTransform(transformToWorldToPack);
            Vector3D axisZ = new Vector3D(syncronizedZup.getM02(), syncronizedZup.getM12(), syncronizedZup.getM22());
            AxisAngle axisAngleFromZUpToVector3D = EuclidGeometryTools.axisAngleFromZUpToVector3D(axisZ);
            axisAngleFromZUpToVector3D.invert();
            
            RigidBodyTransform zaxisToZupTransform = new RigidBodyTransform();
            zaxisToZupTransform.setRotation(axisAngleFromZUpToVector3D);
            transformToWorldToPack.preMultiply(zaxisToZupTransform);
         }
      };
   }
   
//   public static void main(String[] args)
//   {
//      double simulatedYaw = 50.0 / 180.0 * Math.PI;
//      double simulatedPitch = 0.0 / 180.0 * Math.PI;
//      double simulatedRoll = 0.0 / 180.0 * Math.PI;
//      
//      RigidBodyTransform transformToWorldToPack = new RigidBodyTransform();
//      transformToWorldToPack.appendTranslation(0.5, 0.6, 1.0);
//      transformToWorldToPack.appendYawRotation(simulatedYaw);
//      transformToWorldToPack.appendPitchRotation(simulatedPitch);
//      transformToWorldToPack.appendRollRotation(simulatedRoll);
//      
//      RigidBodyTransform transformFromPelvisToRealSense = AtlasSensorInformation.transformPelvisToTrackingCamera;
//      
//      System.out.println("transformToWorldToPack");
//      System.out.println(transformToWorldToPack);
//      transformToWorldToPack.multiply(transformFromPelvisToRealSense);
//      System.out.println(transformToWorldToPack);
//      
//      RigidBodyTransform syncronizedZup = new RigidBodyTransform(transformToWorldToPack);
//      
//      Vector3D axisZ = new Vector3D(syncronizedZup.getM02(), syncronizedZup.getM12(), syncronizedZup.getM22());
//      System.out.println("axisZ");
//      System.out.println(axisZ);
//      
//      AxisAngle axisAngleFromZUpToVector3D = EuclidGeometryTools.axisAngleFromZUpToVector3D(axisZ);
//      
//      System.out.println("axisAngleFromZUpToVector3D");
//      System.out.println(axisAngleFromZUpToVector3D);
//      axisAngleFromZUpToVector3D.invert();
//      System.out.println(axisAngleFromZUpToVector3D);
//
//      RigidBodyTransform zaxisToZupTransform = new RigidBodyTransform();
//      zaxisToZupTransform.setRotation(axisAngleFromZUpToVector3D);
//      System.out.println("zaxisToZupTransform");
//      System.out.println(zaxisToZupTransform);
//      transformToWorldToPack.preMultiply(zaxisToZupTransform);
//
//      System.out.println("transformToWorldToPack");
//      System.out.println(transformToWorldToPack);
//   }
}
