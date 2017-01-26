package us.ihmc.valkyrie.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.lidarScanPublisher.LidarScanPublisher;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.avatar.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.SCSCameraDataReceiver;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
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
   private final PacketCommunicator sensorSuitePacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
         new IHMCCommunicationKryoNetClassList());

   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final FullHumanoidRobotModelFactory fullRobotModelFactory;
   private final LidarScanPublisher lidarScanPublisher;

   public ValkyrieSensorSuiteManager(FullHumanoidRobotModelFactory fullRobotModelFactory, DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         DRCRobotSensorInformation sensorInformation, ValkyrieJointMap jointMap, DRCRobotModel.RobotTarget target)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.sensorInformation = sensorInformation;

      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ValkyrieSensorInformation.MULTISENSE_LIDAR_ID);
      String sensorName = multisenseLidarParameters.getSensorNameInSdf();
      lidarScanPublisher = new LidarScanPublisher(sensorName, fullRobotModelFactory, sensorSuitePacketCommunicator);
      lidarScanPublisher.setPPSTimestampOffsetProvider(ppsTimestampOffsetProvider);
//      lidarScanPublisher.setCollisionBoxProvider(null); // TODO Fill in when we made collision boxes for Val
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(ValkyrieSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      CameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(multisenseLeftEyeCameraParameters.getRobotSide(), fullRobotModelFactory,
            multisenseLeftEyeCameraParameters.getSensorNameInSdf(), robotConfigurationDataBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator,
            ppsTimestampOffsetProvider);
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
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      RosMainNode rosMainNode = new RosMainNode(sensorURI, "darpaRoboticsChallange/networkProcessor");

      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(ValkyrieSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ValkyrieSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(ValkyrieSensorInformation.MULTISENSE_STEREO_ID);
      boolean shouldUseRosParameterSetters = sensorInformation.setupROSParameterSetters();

      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(fullRobotModelFactory, robotConfigurationDataBuffer, rosMainNode,
            sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, multisenseLeftEyeCameraParameters, multisenseLidarParameters, multisenseStereoParameters,
            shouldUseRosParameterSetters);

      lidarScanPublisher.receiveLidarFromROS(multisenseLidarParameters.getRosTopic(), rosMainNode);
      lidarScanPublisher.setScanFrameToWorldFrame();
      lidarScanPublisher.start();

      multiSenseSensorManager.initializeParameterListeners();
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosMainNode.execute();
   }

   @Override
   public void connect() throws IOException
   {
      sensorSuitePacketCommunicator.connect();
   }
}
