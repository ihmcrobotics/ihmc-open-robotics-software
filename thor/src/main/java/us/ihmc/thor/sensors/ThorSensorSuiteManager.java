package us.ihmc.thor.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.avatar.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.SCSCameraDataReceiver;
import us.ihmc.ihmcPerception.depthData.PointCloudDataReceiver;
import us.ihmc.ihmcPerception.depthData.SCSPointCloudLidarReceiver;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.thor.parameters.ThorSensorInformation;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class ThorSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PacketCommunicator sensorSuitePacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
         new IHMCCommunicationKryoNetClassList());


   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final FullHumanoidRobotModelFactory fullRobotModelFactory;

   public ThorSensorSuiteManager(FullHumanoidRobotModelFactory fullRobotModelFactory, DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         DRCRobotSensorInformation sensorInformation, RobotContactPointParameters contactPointParameters, RobotTarget target)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.sensorInformation = sensorInformation;
      if (sensorInformation.getPointCloudParameters().length > 0)
      {
         this.pointCloudDataReceiver = new PointCloudDataReceiver(fullRobotModelFactory, null, ppsTimestampOffsetProvider, contactPointParameters,
               robotConfigurationDataBuffer, sensorSuitePacketCommunicator);
      }
      else
      {
         this.pointCloudDataReceiver = null;
      }
   }

   @Override
   public void initializeSimulatedSensors(ObjectCommunicator scsSensorsCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      CameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(sensorInformation.getCameraParameters(0).getRobotSide(), fullRobotModelFactory, sensorInformation.getCameraParameters(0).getSensorNameInSdf(), robotConfigurationDataBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      if (sensorInformation.getLidarParameters().length > 0)
      {
         new SCSPointCloudLidarReceiver(sensorInformation.getLidarParameters(0).getSensorNameInSdf(), scsSensorsCommunicator, pointCloudDataReceiver);
      }
      cameraDataReceiver.start();
   }

   @Override
   public void initializePhysicalSensors(URI sensorURI)
   {
	   if(sensorURI == null)
	   {
		   throw new IllegalArgumentException("The ros uri was null, thor's physical sensors require a ros uri to be set! Check your Network Parameters.ini file");
	   }
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      RosMainNode rosMainNode = new RosMainNode(sensorURI, "darpaRoboticsChallange/networkProcessor");

      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(ThorSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ThorSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(ThorSensorInformation.MULTISENSE_STEREO_ID);
      boolean shouldUseRosParameterSetters = sensorInformation.setupROSParameterSetters();

      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(fullRobotModelFactory, robotConfigurationDataBuffer,
            rosMainNode, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, multisenseLeftEyeCameraParameters, multisenseLidarParameters,
            multisenseStereoParameters, shouldUseRosParameterSetters);

      multiSenseSensorManager.initializeParameterListeners();
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosMainNode.execute();
   }


   @Override
   public void connect() throws IOException
   {
      sensorSuitePacketCommunicator.connect();
      if (sensorInformation.getLidarParameters().length > 0 || sensorInformation.getPointCloudParameters().length > 0)
      {
         if(pointCloudDataReceiver != null)
         {
            pointCloudDataReceiver.start();
         }
      }
   }
}
