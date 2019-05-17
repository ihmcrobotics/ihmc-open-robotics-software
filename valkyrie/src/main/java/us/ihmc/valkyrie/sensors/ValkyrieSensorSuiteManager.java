package us.ihmc.valkyrie.sensors;

import java.io.IOException;
import java.net.URI;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.lidarScanPublisher.LidarScanPublisher;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher.StereoVisionWorldTransformCalculator;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.avatar.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.SCSCameraDataReceiver;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;

public class ValkyrieSensorSuiteManager implements DRCSensorSuiteManager
{
   private static final boolean ENABLE_STEREO_PUBLISHER = true;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_valkyrie_sensor_suite_node");

   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final FullHumanoidRobotModelFactory fullRobotModelFactory;
   private final LidarScanPublisher lidarScanPublisher;
   private final StereoVisionPointCloudPublisher stereoVisionPointCloudPublisher;

   private final String robotName;

   public ValkyrieSensorSuiteManager(String robotName, FullHumanoidRobotModelFactory fullRobotModelFactory, CollisionBoxProvider collisionBoxProvider,
                                     DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation,
                                     ValkyrieJointMap jointMap, RobotTarget target)
   {
      this.robotName = robotName;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.sensorInformation = sensorInformation;

      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ValkyrieSensorInformation.MULTISENSE_LIDAR_ID);
      String sensorName = multisenseLidarParameters.getSensorNameInSdf();
      String rcdTopicName = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName).generateTopicName(RobotConfigurationData.class);
      lidarScanPublisher = new LidarScanPublisher(sensorName, fullRobotModelFactory, ros2Node, rcdTopicName);
      lidarScanPublisher.setPPSTimestampOffsetProvider(ppsTimestampOffsetProvider);
      lidarScanPublisher.setCollisionBoxProvider(collisionBoxProvider);

      if (ENABLE_STEREO_PUBLISHER)
      {
         stereoVisionPointCloudPublisher = new StereoVisionPointCloudPublisher(fullRobotModelFactory, ros2Node, rcdTopicName);
         stereoVisionPointCloudPublisher.setPPSTimestampOffsetProvider(ppsTimestampOffsetProvider);
         stereoVisionPointCloudPublisher.setCustomStereoVisionTransformer(createCustomStereoTransformCalculator());
      }
      else
      {
         stereoVisionPointCloudPublisher = null;
      }
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(ValkyrieSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      CameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(multisenseLeftEyeCameraParameters.getRobotSide(), fullRobotModelFactory,
                                                                        multisenseLeftEyeCameraParameters.getSensorNameInSdf(), robotConfigurationDataBuffer,
                                                                        scsSensorsCommunicator, ros2Node, ppsTimestampOffsetProvider);
      cameraDataReceiver.start();

      lidarScanPublisher.receiveLidarFromSCS(scsSensorsCommunicator);
      lidarScanPublisher.setScanFrameToLidarSensorFrame();
      lidarScanPublisher.start();
   }

   @Override
   public void initializePhysicalSensors(URI sensorURI)
   {
      if (sensorURI == null)
      {
         throw new IllegalArgumentException("The ros uri was null, val's physical sensors require a ros uri to be set! Check your Network Parameters.ini file");
      }
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));

      RosMainNode rosMainNode = new RosMainNode(sensorURI, "darpaRoboticsChallange/networkProcessor");

      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(ValkyrieSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ValkyrieSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(ValkyrieSensorInformation.MULTISENSE_STEREO_ID);
      boolean shouldUseRosParameterSetters = sensorInformation.setupROSParameterSetters();

      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(fullRobotModelFactory, robotConfigurationDataBuffer, rosMainNode, ros2Node,
                                                                                    ppsTimestampOffsetProvider, multisenseLeftEyeCameraParameters,
                                                                                    multisenseLidarParameters, multisenseStereoParameters,
                                                                                    shouldUseRosParameterSetters);

      lidarScanPublisher.receiveLidarFromROS(multisenseLidarParameters.getRosTopic(), rosMainNode);
      lidarScanPublisher.setScanFrameToWorldFrame();
      lidarScanPublisher.start();

      if (ENABLE_STEREO_PUBLISHER)
      {
         stereoVisionPointCloudPublisher.receiveStereoPointCloudFromROS(multisenseStereoParameters.getRosTopic(), rosMainNode);
         stereoVisionPointCloudPublisher.start();
      }

      multiSenseSensorManager.initializeParameterListeners();
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosMainNode.execute();
   }

   @Override
   public void connect() throws IOException
   {
   }

   private StereoVisionWorldTransformCalculator createCustomStereoTransformCalculator()
   {
      return new StereoVisionWorldTransformCalculator()
      {
         private final RigidBodyTransform transformFromHeadToUpperNeckPitchLink = ValkyrieSensorInformation.getTransformFromHeadToUpperNeckPitchLink();

         @Override
         public void computeTransformToWorld(FullRobotModel fullRobotModel, ReferenceFrame scanPointsFrame, RigidBodyTransform transformToWorldToPack, Pose3DBasics sensorPoseToPack)
         {
            ReferenceFrame neckFrame = fullRobotModel.getHeadBaseFrame();
            neckFrame.getTransformToDesiredFrame(transformToWorldToPack, ReferenceFrame.getWorldFrame());
            transformToWorldToPack.multiply(transformFromHeadToUpperNeckPitchLink);
            sensorPoseToPack.set(transformToWorldToPack);
         }
      };
   }
}
