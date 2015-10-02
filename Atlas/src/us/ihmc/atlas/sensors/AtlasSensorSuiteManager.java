package us.ihmc.atlas.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.FisheyeCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.darpaRoboticsChallenge.sensors.blackfly.BlackFlyParameterSetter;
import us.ihmc.darpaRoboticsChallenge.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PacketCommunicator sensorSuitePacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
         new IHMCCommunicationKryoNetClassList());

   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final SDFFullRobotModelFactory modelFactory;
   private SideDependentList<BlackFlyParameterSetter> blackFlyParameterSetters = null;

   public AtlasSensorSuiteManager(SDFFullRobotModelFactory modelFactory, CollisionBoxProvider collisionBoxProvider,
         DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation, DRCRobotJointMap jointMap,
         AtlasPhysicalProperties physicalProperties, DRCRobotModel.RobotTarget targetDeployment)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
      this.robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      this.pointCloudDataReceiver = new PointCloudDataReceiver(modelFactory, collisionBoxProvider, ppsTimestampOffsetProvider, jointMap,
            robotConfigurationDataBuffer, sensorSuitePacketCommunicator);
      this.modelFactory = modelFactory;
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      SCSCameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(sensorInformation.getCameraParameters(0).getRobotSide(), modelFactory,
            sensorInformation.getCameraParameters(0).getSensorNameInSdf(), robotConfigurationDataBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator,
            ppsTimestampOffsetProvider);
      cameraDataReceiver.start();

      //      if (sensorInformation.getPointCloudParameters().length > 0)
      //      {
      //         new SCSPointCloudDataReceiver(depthDataProcessor, robotPoseBuffer, scsCommunicator);
      //      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         new SCSPointCloudLidarReceiver(sensorInformation.getLidarParameters(0).getSensorNameInSdf(), scsSensorsCommunicator, pointCloudDataReceiver);
      }

      //      if (DRCConfigParameters.CALIBRATE_ARM_MODE)
      //      {
      //         ArmCalibrationHelper armCalibrationHelper = new ArmCalibrationHelper(sensorSuitePacketCommunicator, jointMap);
      //         cameraReceiver.registerCameraListener(armCalibrationHelper);
      //      }

      //      IMUBasedHeadPoseCalculatorFactory.create(sensorSuitePacketCommunicator, sensorInformation);

      VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(sensorSuitePacketCommunicator, pointCloudDataReceiver, modelFactory,
            robotConfigurationDataBuffer, false);
      cameraDataReceiver.registerCameraListener(visionPoseEstimator);
   }

   @Override
   public void initializePhysicalSensors(URI rosCoreURI)
   {
      if (rosCoreURI == null)
         throw new RuntimeException(getClass().getSimpleName() + " Physical sensor requires rosURI to be set in " + NetworkParameters.defaultParameterFile);

      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      ForceSensorNoiseEstimator forceSensorNoiseEstimator = new ForceSensorNoiseEstimator(sensorSuitePacketCommunicator);
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, forceSensorNoiseEstimator);

      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, "atlas/sensorSuiteManager", true);

      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);

      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(modelFactory, pointCloudDataReceiver, robotConfigurationDataBuffer,
            rosMainNode, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, rosCoreURI, multisenseLeftEyeCameraParameters, multisenseLidarParameters,
            multisenseStereoParameters, sensorInformation.setupROSParameterSetters());

      DRCRobotCameraParameters leftFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_LEFT_CAMERA_ID);
      DRCRobotCameraParameters rightFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_RIGHT_CAMERA_ID);

      FisheyeCameraReceiver leftFishEyeCameraReceiver = new FisheyeCameraReceiver(modelFactory, leftFishEyeCameraParameters, robotConfigurationDataBuffer,
            sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, rosMainNode);
      FisheyeCameraReceiver rightFishEyeCameraReceiver = new FisheyeCameraReceiver(modelFactory, rightFishEyeCameraParameters, robotConfigurationDataBuffer,
            sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, rosMainNode);

      leftFishEyeCameraReceiver.start();
      rightFishEyeCameraReceiver.start();

      VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(sensorSuitePacketCommunicator, pointCloudDataReceiver, modelFactory,
            robotConfigurationDataBuffer, true);
      multiSenseSensorManager.registerCameraListener(visionPoseEstimator);

      blackFlyParameterSetters = new SideDependentList<BlackFlyParameterSetter>();
      for (RobotSide side : RobotSide.values)
         blackFlyParameterSetters.put(side, new BlackFlyParameterSetter(rosMainNode, side, "/" + side.getLowerCaseName() + "/camera/camera_nodelet",
               sensorSuitePacketCommunicator));

      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);

      //      if (DRCConfigParameters.CALIBRATE_ARM_MODE)
      //      {
      //         ArmCalibrationHelper armCalibrationHelper = new ArmCalibrationHelper(sensorSuitePacketCommunicator, jointMap);
      //         multiSenseSensorManager.registerCameraListener(armCalibrationHelper);
      //      }

      multiSenseSensorManager.initializeParameterListeners();

      //      IMUBasedHeadPoseCalculatorFactory.create(sensorSuitePacketCommunicator, sensorInformation, rosMainNode);
      rosMainNode.execute();
   }

   @Override
   public void connect() throws IOException
   {
      sensorSuitePacketCommunicator.connect();
      if (sensorInformation.getLidarParameters().length > 0)
      {
         pointCloudDataReceiver.start();

      }
   }
}
