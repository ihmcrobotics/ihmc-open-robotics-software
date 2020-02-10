package us.ihmc.atlas.sensors;

import java.io.IOException;
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
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.ihmcPerception.camera.FisheyeCameraReceiver;
import us.ihmc.ihmcPerception.camera.SCSCameraDataReceiver;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
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
   private final AtlasPointCloudSensorManager pointCloudSensorManager;

   private static final boolean ENABLE_STEREO_PUBLISHER = false;
   private static final boolean ENABLE_DEPTH_PUBLISHER = true;
   private static final boolean USE_DEPTH_FRAME_ESTIMATED_BY_TRACKING = false;

   private final RobotROSClockCalculator rosClockCalculator;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final FullHumanoidRobotModelFactory modelFactory;

   private final String robotName;

   private SCSCameraDataReceiver cameraDataReceiver;

   private RosMainNode rosMainNode;

   private MultiSenseSensorManager multiSenseSensorManager;

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

      pointCloudSensorManager = new AtlasPointCloudSensorManager(modelFactory,
                                                                 ros2Node,
                                                                 rcdTopicName,
                                                                 rosClockCalculator,
                                                                 USE_DEPTH_FRAME_ESTIMATED_BY_TRACKING);
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           RobotConfigurationData.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

      cameraDataReceiver = new SCSCameraDataReceiver(sensorInformation.getCameraParameters(0).getRobotSide(),
                                                     modelFactory,
                                                     sensorInformation.getCameraParameters(0).getSensorNameInSdf(),
                                                     robotConfigurationDataBuffer,
                                                     scsSensorsCommunicator,
                                                     ros2Node,
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

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           RobotConfigurationData.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

      rosMainNode = new RosMainNode(rosCoreURI, "atlas/sensorSuiteManager", true);

      AvatarRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      AvatarRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);

      lidarScanPublisher.receiveLidarFromROSAsPointCloud2WithSource(multisenseLidarParameters.getRosTopic(), rosMainNode);
      lidarScanPublisher.setScanFrameToWorldFrame();

      if (ENABLE_STEREO_PUBLISHER)
         multisenseStereoVisionPointCloudPublisher.receiveStereoPointCloudFromROS1(multisenseStereoParameters.getRosTopic(), rosMainNode);
      if (ENABLE_DEPTH_PUBLISHER)
         pointCloudSensorManager.receiveDataFromROS1(rosMainNode);

      multiSenseSensorManager = new MultiSenseSensorManager(modelFactory,
                                                                                    robotConfigurationDataBuffer,
                                                                                    rosMainNode,
                                                                                    ros2Node,
                                                                                    rosClockCalculator,
                                                                                    multisenseLeftEyeCameraParameters,
                                                                                    multisenseLidarParameters,
                                                                                    multisenseStereoParameters,
                                                                                    sensorInformation.setupROSParameterSetters());

      AvatarRobotCameraParameters leftFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_LEFT_CAMERA_ID);
      AvatarRobotCameraParameters rightFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_RIGHT_CAMERA_ID);

      FisheyeCameraReceiver leftFishEyeCameraReceiver = new FisheyeCameraReceiver(modelFactory,
                                                                                  leftFishEyeCameraParameters,
                                                                                  robotConfigurationDataBuffer,
                                                                                  ros2Node,
                                                                                  rosClockCalculator::computeRobotMonotonicTime,
                                                                                  rosMainNode);
      FisheyeCameraReceiver rightFishEyeCameraReceiver = new FisheyeCameraReceiver(modelFactory,
                                                                                   rightFishEyeCameraParameters,
                                                                                   robotConfigurationDataBuffer,
                                                                                   ros2Node,
                                                                                   rosClockCalculator::computeRobotMonotonicTime,
                                                                                   rosMainNode);

      leftFishEyeCameraReceiver.start();
      rightFishEyeCameraReceiver.start();
      lidarScanPublisher.start();

      if (ENABLE_STEREO_PUBLISHER)
         multisenseStereoVisionPointCloudPublisher.start();
      if (ENABLE_DEPTH_PUBLISHER)
         pointCloudSensorManager.start();

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

   @Override
   public void closeAndDispose()
   {
      lidarScanPublisher.shutdown();
      if (ENABLE_STEREO_PUBLISHER)
         multisenseStereoVisionPointCloudPublisher.shutdown();
      if (ENABLE_DEPTH_PUBLISHER)
         pointCloudSensorManager.shutdown();
      if (cameraDataReceiver != null)
         cameraDataReceiver.closeAndDispose();
      if (multiSenseSensorManager != null)
         multiSenseSensorManager.closeAndDispose();
      if (rosMainNode != null)
         rosMainNode.shutdown();
      ros2Node.destroy();
   }
}
