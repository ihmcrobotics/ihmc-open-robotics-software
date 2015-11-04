package us.ihmc.valkyrie.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModelFactory;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.darpaRoboticsChallenge.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
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
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final SDFFullHumanoidRobotModelFactory fullRobotModelFactory;

   public ValkyrieSensorSuiteManager(SDFFullHumanoidRobotModelFactory fullRobotModelFactory, DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         DRCRobotSensorInformation sensorInformation, ValkyrieJointMap jointMap, DRCRobotModel.RobotTarget target)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.sensorInformation = sensorInformation;
      if(sensorInformation.getPointCloudParameters().length>0)
      {
         this.pointCloudDataReceiver = new PointCloudDataReceiver(fullRobotModelFactory, null, ppsTimestampOffsetProvider, jointMap, robotConfigurationDataBuffer, sensorSuitePacketCommunicator);
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
		   throw new IllegalArgumentException("The ros uri was null, val's physical sensors require a ros uri to be set! Check your Network Parameters.ini file");
	   }
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);
      
      RosMainNode rosMainNode = new RosMainNode(sensorURI, "darpaRoboticsChallange/networkProcessor");

      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(ValkyrieSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(ValkyrieSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(ValkyrieSensorInformation.MULTISENSE_STEREO_ID);
      boolean shouldUseRosParameterSetters = sensorInformation.setupROSParameterSetters();
      
      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(fullRobotModelFactory, pointCloudDataReceiver, robotConfigurationDataBuffer,
            rosMainNode, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, sensorURI, multisenseLeftEyeCameraParameters, multisenseLidarParameters,
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
