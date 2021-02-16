package us.ihmc.valkyrie.sensors;

import java.net.URI;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.lidarScanPublisher.LidarScanPublisher;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher.StereoVisionWorldTransformCalculator;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.avatar.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.SCSCameraDataReceiver;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

public class ValkyrieSensorSuiteManager implements DRCSensorSuiteManager
{
   public static final String NODE_NAME = "ihmc_valkyrie_sensor_suite_node";
   private final ROS2NodeInterface ros2Node;

   private final String robotName;
   private final CollisionBoxProvider collisionBoxProvider;
   private final RobotROSClockCalculator rosClockCalculator;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final FullHumanoidRobotModelFactory fullRobotModelFactory;

   private CameraDataReceiver cameraDataReceiver;
   private MultiSenseSensorManager multiSenseSensorManager;
   private LidarScanPublisher lidarScanPublisher;
   private StereoVisionPointCloudPublisher stereoVisionPointCloudPublisher;

   private RosMainNode rosMainNode;

   private boolean enableVideoPublisher = true;
   private boolean enableLidarScanPublisher = true;
   private boolean enableStereoVisionPointCloudPublisher = true;

   public ValkyrieSensorSuiteManager(String robotName,
                                     FullHumanoidRobotModelFactory fullRobotModelFactory,
                                     CollisionBoxProvider collisionBoxProvider,
                                     RobotROSClockCalculator rosClockCalculator,
                                     HumanoidRobotSensorInformation sensorInformation,
                                     ValkyrieJointMap jointMap,
                                     RobotTarget target,
                                     ROS2NodeInterface ros2Node)
   {
      this.robotName = robotName;
      this.collisionBoxProvider = collisionBoxProvider;
      this.rosClockCalculator = rosClockCalculator;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.sensorInformation = sensorInformation;
      this.ros2Node = ros2Node == null ? ROS2Tools.createROS2Node(FAST_RTPS, NODE_NAME) : ros2Node;
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

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      if (enableVideoPublisher)
      {
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                       RobotConfigurationData.class, ROS2Tools.getControllerOutputTopic(robotName),
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

         AvatarRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(ValkyrieSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);

         cameraDataReceiver = new SCSCameraDataReceiver(multisenseLeftEyeCameraParameters.getRobotSide(),
                                                        fullRobotModelFactory,
                                                        multisenseLeftEyeCameraParameters.getSensorNameInSdf(),
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
   public void initializePhysicalSensors(URI sensorURI)
   {
      if (sensorURI == null)
      {
         throw new IllegalArgumentException("The ros uri was null, val's physical sensors require a ros uri to be set! Check your Network Parameters.ini file");
      }

      rosMainNode = new RosMainNode(sensorURI, "darpaRoboticsChallange/networkProcessor");

      if (enableVideoPublisher)
      {
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                       RobotConfigurationData.class, ROS2Tools.getControllerOutputTopic(robotName),
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

         AvatarRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(ValkyrieSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
         AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ValkyrieSensorInformation.MULTISENSE_LIDAR_ID);
         AvatarRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(ValkyrieSensorInformation.MULTISENSE_STEREO_ID);
         boolean shouldUseRosParameterSetters = sensorInformation.setupROSParameterSetters();

         multiSenseSensorManager = new MultiSenseSensorManager(fullRobotModelFactory,
                                                               robotConfigurationDataBuffer,
                                                               rosMainNode,
                                                               ros2Node,
                                                               rosClockCalculator,
                                                               multisenseLeftEyeCameraParameters,
                                                               multisenseLidarParameters,
                                                               multisenseStereoParameters,
                                                               shouldUseRosParameterSetters);
      }

      if (enableLidarScanPublisher)
      {
         AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ValkyrieSensorInformation.MULTISENSE_LIDAR_ID);
         lidarScanPublisher = createLidarScanPublisher();
         lidarScanPublisher.receiveLidarFromROS(multisenseLidarParameters.getRosTopic(), rosMainNode);
         lidarScanPublisher.setPublisherPeriodInMillisecond(25L);
         lidarScanPublisher.setScanFrameToWorldFrame();
      }

      if (enableStereoVisionPointCloudPublisher)
      {
         AvatarRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(ValkyrieSensorInformation.MULTISENSE_STEREO_ID);
         stereoVisionPointCloudPublisher = createStereoPointCloudPublisher();
         stereoVisionPointCloudPublisher.setFilterThreshold(ValkyrieSensorInformation.linearVelocityThreshold,
                                                            ValkyrieSensorInformation.angularVelocityThreshold);
         stereoVisionPointCloudPublisher.enableFilter(true);
         stereoVisionPointCloudPublisher.receiveStereoPointCloudFromROS1(multisenseStereoParameters.getRosTopic(), rosMainNode);
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
      if (stereoVisionPointCloudPublisher != null)
         stereoVisionPointCloudPublisher.start();
      if (rosMainNode != null)
         rosMainNode.execute();
   }

   private StereoVisionPointCloudPublisher createStereoPointCloudPublisher()
   {
      ROS2Topic rcdTopicName = ROS2Tools.getControllerOutputTopic(robotName).withTypeName(RobotConfigurationData.class);

      StereoVisionPointCloudPublisher publisher = new StereoVisionPointCloudPublisher(fullRobotModelFactory, ros2Node, rcdTopicName);
      publisher.setROSClockCalculator(rosClockCalculator);
      publisher.setCustomStereoVisionTransformer(createCustomStereoTransformCalculator());
      return publisher;
   }

   private LidarScanPublisher createLidarScanPublisher()
   {
      AvatarRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ValkyrieSensorInformation.MULTISENSE_LIDAR_ID);
      String sensorName = multisenseLidarParameters.getSensorNameInSdf();
      LidarScanPublisher publisher = new LidarScanPublisher(sensorName, fullRobotModelFactory, ros2Node);
      publisher.setROSClockCalculator(rosClockCalculator);
      publisher.setShadowFilter();
      publisher.setSelfCollisionFilter(collisionBoxProvider);
      return publisher;
   }

   private StereoVisionWorldTransformCalculator createCustomStereoTransformCalculator()
   {
      return new StereoVisionWorldTransformCalculator()
      {
         private final RigidBodyTransform transformFromHeadToUpperNeckPitchLink = ValkyrieSensorInformation.getTransformFromHeadToUpperNeckPitchLink();

         @Override
         public void computeTransformToWorld(FullRobotModel fullRobotModel, RigidBodyTransform transformToWorldToPack, Pose3DBasics sensorPoseToPack)
         {
            ReferenceFrame neckFrame = fullRobotModel.getHeadBaseFrame();
            neckFrame.getTransformToDesiredFrame(transformToWorldToPack, ReferenceFrame.getWorldFrame());
            transformToWorldToPack.multiply(transformFromHeadToUpperNeckPitchLink);
            sensorPoseToPack.set(transformToWorldToPack);
         }
      };
   }

   public MultiSenseSensorManager getMultiSenseSensorManager()
   {
      return multiSenseSensorManager;
   }

   public LidarScanPublisher getLidarScanPublisher()
   {
      return lidarScanPublisher;
   }

   public StereoVisionPointCloudPublisher getStereoVisionPointCloudPublisher()
   {
      return stereoVisionPointCloudPublisher;
   }

   @Override
   public void closeAndDispose()
   {
      if (lidarScanPublisher != null)
         lidarScanPublisher.shutdown();
      if (stereoVisionPointCloudPublisher != null)
         stereoVisionPointCloudPublisher.shutdown();
      if (cameraDataReceiver != null)
         cameraDataReceiver.closeAndDispose();
      if (multiSenseSensorManager != null)
         multiSenseSensorManager.closeAndDispose();
      if (rosMainNode != null)
         rosMainNode.shutdown();
      if (ros2Node.getName().equals(NODE_NAME)) // i.e. we created this node, so should manage it
      {
         ((ROS2Node) ros2Node).destroy();
      }
   }
}
