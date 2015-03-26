package us.ihmc.valkyrie.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.sensing.CameraInformationPacket;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.RosPointCloudReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.wholeBodyController.DRCHandType;

public class ValkyrieSensorSuiteManager implements DRCSensorSuiteManager
{
   private final KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.SENSOR_MANAGER.ordinal(), "VAL_SENSOR_MANAGER");
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final SDFFullRobotModelFactory fullRobotModelFactory;

   public ValkyrieSensorSuiteManager(SDFFullRobotModelFactory fullRobotModelFactory, PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         DRCRobotSensorInformation sensorInformation, ValkyrieJointMap jointMap, boolean runningOnRealRobot)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.sensorInformation = sensorInformation;
      this.pointCloudDataReceiver = new PointCloudDataReceiver(fullRobotModelFactory, DRCHandType.VALKYRIE, ppsTimestampOffsetProvider, jointMap, robotConfigurationDataBuffer, sensorSuitePacketCommunicator);
   }

   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsSensorsCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      CameraDataReceiver cameraDataReceiver = new SCSCameraDataReceiver(fullRobotModelFactory, sensorInformation.getCameraParameters(0).getSensorNameInSdf(), robotConfigurationDataBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      if (sensorInformation.getLidarParameters().length > 0)
      {
         new SCSPointCloudLidarReceiver(sensorInformation.getLidarParameters(0).getSensorNameInSdf(), scsSensorsCommunicator, pointCloudDataReceiver);
      }
      pointCloudDataReceiver.start();
      cameraDataReceiver.start();
   }

   @Override
   public void initializePhysicalSensors(URI sensorURI)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);

      RosMainNode rosMainNode = new RosMainNode(sensorURI, "darpaRoboticsChallange/networkProcessor");

      DRCRobotCameraParameters cameraParamaters = sensorInformation.getCameraParameters(0);

      CameraDataReceiver cameraDataReceiver = new RosCameraReceiver(fullRobotModelFactory, sensorInformation.getCameraParameters(0), robotConfigurationDataBuffer, rosMainNode, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider, null, sensorURI);
      cameraDataReceiver.start();
      
      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(cameraParamaters, rosMainNode, sensorSuitePacketCommunicator, null);
      sensorSuitePacketCommunicator.attachListener(CameraInformationPacket.class, cameraInfoServer);

      //TODO: Fixme
      //      new IbeoPointCloudDataReceiver(rosMainNode, robotPoseBuffer, sensorSuitePacketCommunicator, sdfFullRobotModel, sensorInformation.getPointCloudParameters(ValkyrieSensorInformation.IBEO_ID), lidarDataFilter);
      new RosPointCloudReceiver(sensorInformation.getLidarParameters(0), rosMainNode, ReferenceFrame.getWorldFrame(), pointCloudDataReceiver);
      pointCloudDataReceiver.start();
      
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosMainNode.execute();
   }

   @Override
   public PacketCommunicator getProcessedSensorsCommunicator()
   {
      return sensorSuitePacketCommunicator;
   }
}
