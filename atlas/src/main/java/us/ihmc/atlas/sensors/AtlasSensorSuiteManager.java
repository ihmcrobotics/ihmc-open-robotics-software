package us.ihmc.atlas.sensors;

import java.net.URI;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.RobotPhysicalProperties;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.lidarScanPublisher.LidarScanPublisher;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.avatar.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.ihmcPerception.camera.FisheyeCameraReceiver;
import us.ihmc.ihmcPerception.camera.SCSCameraDataReceiver;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.ros2.ROS2Topic;
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
   private static final boolean USE_DEPTH_FRAME_ESTIMATED_BY_TRACKING = false;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_atlas_sensor_suite_node");

   private final String robotName;
   private final FullHumanoidRobotModelFactory modelFactory;
   private final CollisionBoxProvider collisionBoxProvider;
   private final RobotROSClockCalculator rosClockCalculator;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;

   private SCSCameraDataReceiver cameraDataReceiver;
   private MultiSenseSensorManager multiSenseSensorManager;
   private LidarScanPublisher lidarScanPublisher;
   private StereoVisionPointCloudPublisher multisenseStereoVisionPointCloudPublisher;
   private AtlasPointCloudSensorManager pointCloudSensorManager;
   private FisheyeCameraReceiver leftFishEyeCameraReceiver;
   private FisheyeCameraReceiver rightFishEyeCameraReceiver;

   private RosMainNode rosMainNode;

   private boolean enableVideoPublisher = true;
   private boolean enableLidarScanPublisher = true;
   private boolean enableStereoVisionPointCloudPublisher = false;
   private boolean enableDepthPointCloudPublisher = true;
   private boolean enableFisheyeCameraPublishers = false;

   public AtlasSensorSuiteManager(String robotName,
                                  FullHumanoidRobotModelFactory modelFactory,
                                  CollisionBoxProvider collisionBoxProvider,
                                  RobotROSClockCalculator rosClockCalculator,
                                  HumanoidRobotSensorInformation sensorInformation,
                                  DRCRobotJointMap jointMap,
                                  RobotPhysicalProperties physicalProperties,
                                  RobotTarget targetDeployment)
   {
      this.robotName = robotName;
      this.collisionBoxProvider = collisionBoxProvider;
      this.rosClockCalculator = rosClockCalculator;
      this.sensorInformation = sensorInformation;
      this.robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      this.modelFactory = modelFactory;
   }

   public void setEnableVideoPublisher(boolean enableVideoPublisher)
   {
      this.enableVideoPublisher = enableVideoPublisher;
   }

   public void setEnableLidarScanPublisher(boolean enableLidarScanPublisher)
   {
      this.enableLidarScanPublisher = enableLidarScanPublisher;
   }

   public void setEnableStereoVisionPointCloudPublisher(boolean enableStereoVisionPointCloudPublisher)
   {
      this.enableStereoVisionPointCloudPublisher = enableStereoVisionPointCloudPublisher;
   }

   public void setEnableDepthPointCloudPublisher(boolean enableDepthPointCloudPublisher)
   {
      this.enableDepthPointCloudPublisher = enableDepthPointCloudPublisher;
   }

   public void setEnableFisheyeCameraPublishers(boolean enableFisheyeCameraPublishers)
   {
      this.enableFisheyeCameraPublishers = enableFisheyeCameraPublishers;
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      if (enableVideoPublisher)
      {
         ROS2Tools.createCallbackSubscription(ros2Node,
                                              ROS2Tools.getRobotConfigurationDataTopic(robotName),
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

         cameraDataReceiver = new SCSCameraDataReceiver(sensorInformation.getCameraParameters(0).getRobotSide(),
                                                        modelFactory,
                                                        sensorInformation.getCameraParameters(0).getSensorNameInSdf(),
                                                        robotConfigurationDataBuffer,
                                                        scsSensorsCommunicator,
                                                        ros2Node,
                                                        rosClockCalculator::computeRobotMonotonicTime);
      }

      if (enableLidarScanPublisher)
      {
         lidarScanPublisher = createLidarScanPublisher();
         lidarScanPublisher.receiveLidarFromSCS(scsSensorsCommunicator);
         lidarScanPublisher.setScanFrameToLidarSensorFrame();
      }
   }

   @Override
   public void initializePhysicalSensors(URI rosCoreURI)
   {
      if (rosCoreURI == null)
         throw new RuntimeException(getClass().getSimpleName() + " Physical sensor requires rosURI to be set in " + NetworkParameters.defaultParameterFile);

      rosMainNode = new RosMainNode(rosCoreURI, "atlas/sensorSuiteManager", true);

      if (enableVideoPublisher)
      {
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                       RobotConfigurationData.class, ROS2Tools.getControllerOutputTopic(robotName),
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

         AvatarRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
         AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
         AvatarRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);

         multiSenseSensorManager = new MultiSenseSensorManager(modelFactory,
                                                               robotConfigurationDataBuffer,
                                                               rosMainNode,
                                                               ros2Node,
                                                               rosClockCalculator,
                                                               multisenseLeftEyeCameraParameters,
                                                               multisenseLidarParameters,
                                                               multisenseStereoParameters,
                                                               sensorInformation.setupROSParameterSetters());
      }

      if (enableLidarScanPublisher)
      {
         AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
         lidarScanPublisher = createLidarScanPublisher();
         lidarScanPublisher.receiveLidarFromROSAsPointCloud2WithSource(multisenseLidarParameters.getRosTopic(), rosMainNode);
         lidarScanPublisher.setScanFrameToWorldFrame();
      }

      if (enableStereoVisionPointCloudPublisher)
      {
         multisenseStereoVisionPointCloudPublisher = new StereoVisionPointCloudPublisher(modelFactory, ros2Node, ROS2Tools.MULTISENSE_STEREO_POINT_CLOUD);
         multisenseStereoVisionPointCloudPublisher.setROSClockCalculator(rosClockCalculator);
         AvatarRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);
         multisenseStereoVisionPointCloudPublisher.receiveStereoPointCloudFromROS1(multisenseStereoParameters.getRosTopic(), rosMainNode);
      }

      if (enableDepthPointCloudPublisher)
      {
         pointCloudSensorManager = new AtlasPointCloudSensorManager(modelFactory,
                                                                    ros2Node,
                                                                    rosClockCalculator,
                                                                    USE_DEPTH_FRAME_ESTIMATED_BY_TRACKING);
         pointCloudSensorManager.setCollisionBoxProvider(collisionBoxProvider);
         pointCloudSensorManager.receiveDataFromROS1(rosMainNode);
      }

      if (enableFisheyeCameraPublishers)
      {
         AvatarRobotCameraParameters leftFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_LEFT_CAMERA_ID);
         AvatarRobotCameraParameters rightFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_RIGHT_CAMERA_ID);

         leftFishEyeCameraReceiver = new FisheyeCameraReceiver(modelFactory,
                                                               leftFishEyeCameraParameters,
                                                               robotConfigurationDataBuffer,
                                                               ros2Node,
                                                               rosClockCalculator::computeRobotMonotonicTime,
                                                               rosMainNode);
         rightFishEyeCameraReceiver = new FisheyeCameraReceiver(modelFactory,
                                                                rightFishEyeCameraParameters,
                                                                robotConfigurationDataBuffer,
                                                                ros2Node,
                                                                rosClockCalculator::computeRobotMonotonicTime,
                                                                rosMainNode);
      }

      rosClockCalculator.setROSMainNode(rosMainNode);
   }

   @Override
   public void connect()
   {
      if (cameraDataReceiver != null)
         cameraDataReceiver.start();
      if (multiSenseSensorManager != null)
      {
         multiSenseSensorManager.initializeParameterListeners();
         multiSenseSensorManager.start();
      }
      if (lidarScanPublisher != null)
         lidarScanPublisher.start();
      if (multisenseStereoVisionPointCloudPublisher != null)
         multisenseStereoVisionPointCloudPublisher.start();
      if (pointCloudSensorManager != null)
         pointCloudSensorManager.start();
      if (leftFishEyeCameraReceiver != null)
         leftFishEyeCameraReceiver.start();
      if (rightFishEyeCameraReceiver != null)
         rightFishEyeCameraReceiver.start();

      if (rosMainNode != null)
         rosMainNode.execute();
   }

   @Override
   public void closeAndDispose()
   {
      if (lidarScanPublisher != null)
         lidarScanPublisher.shutdown();
      if (multisenseStereoVisionPointCloudPublisher != null)
         multisenseStereoVisionPointCloudPublisher.shutdown();
      if (pointCloudSensorManager != null)
         pointCloudSensorManager.shutdown();
      if (cameraDataReceiver != null)
         cameraDataReceiver.closeAndDispose();
      if (multiSenseSensorManager != null)
         multiSenseSensorManager.closeAndDispose();
      if (rosMainNode != null)
         rosMainNode.shutdown();
      if (leftFishEyeCameraReceiver != null)
         leftFishEyeCameraReceiver.closeAndDispose();
      if (rightFishEyeCameraReceiver != null)
         rightFishEyeCameraReceiver.closeAndDispose();
      ros2Node.destroy();
   }

   private LidarScanPublisher createLidarScanPublisher()
   {
      AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      String sensorName = multisenseLidarParameters.getSensorNameInSdf();
      ROS2Topic rcdTopicName = ROS2Tools.getControllerOutputTopic(robotName).withType(RobotConfigurationData.class);
      LidarScanPublisher lidarScanPublisher = new LidarScanPublisher(sensorName, modelFactory, ros2Node, rcdTopicName);
      lidarScanPublisher.setROSClockCalculator(rosClockCalculator);
      lidarScanPublisher.setShadowFilter();
      lidarScanPublisher.setSelfCollisionFilter(collisionBoxProvider);
      return lidarScanPublisher;
   }

   public LidarScanPublisher getLidarScanPublisher()
   {
      return lidarScanPublisher;
   }

   public StereoVisionPointCloudPublisher getMultisenseStereoVisionPointCloudPublisher()
   {
      return multisenseStereoVisionPointCloudPublisher;
   }

   public MultiSenseSensorManager getMultiSenseSensorManager()
   {
      return multiSenseSensorManager;
   }
}
