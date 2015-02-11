package us.ihmc.valkyrie.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.sensing.CameraInformationPacket;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.darpaRoboticsChallenge.sensors.ibeo.IbeoPointCloudDataReceiver;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.ihmcPerception.depthData.RobotBoundingBoxes;
import us.ihmc.ros.jni.wrapper.RosNativeNetworkProcessor;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.wholeBodyController.DRCHandType;

public class ValkyrieSensorSuiteManager implements DRCSensorSuiteManager
{
   private final KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),PacketDestination.SENSOR_MANAGER.ordinal(), "VAL_SENSOR_MANAGER");
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;
   private final SDFFullRobotModel sdfFullRobotModel;
   private final RobotDataReceiver drcRobotDataReceiver;
   private final DepthDataFilter lidarDataFilter;
   private final RobotBoundingBoxes robotBoundingBoxes;
   private RobotPoseBuffer robotPoseBuffer;
   private DepthDataProcessor depthDataProcessor;
   

   public ValkyrieSensorSuiteManager(PPSTimestampOffsetProvider ppsTimestampOffsetProvider, SDFFullRobotModel sdfFullRobotModel, DRCRobotSensorInformation sensorInformation, boolean runningOnRealRobot)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.drcRobotDataReceiver = new RobotDataReceiver(sdfFullRobotModel, null, true);
      this.robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, DRCHandType.VALKYRIE, sdfFullRobotModel);
      this.lidarDataFilter = new DepthDataFilter(robotBoundingBoxes, sdfFullRobotModel);
   }

   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsSensorsCommunicator)
   {
      robotPoseBuffer = new RobotPoseBuffer(sensorSuitePacketCommunicator, 10000, timestampProvider);
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);
      
      this.depthDataProcessor = new DepthDataProcessor(sensorSuitePacketCommunicator,lidarDataFilter);
      new SCSCameraDataReceiver(robotPoseBuffer, scsSensorsCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      new SCSPointCloudDataReceiver(robotPoseBuffer, scsSensorsCommunicator, sdfFullRobotModel, sensorInformation, ppsTimestampOffsetProvider, lidarDataFilter);
   }

   @Override
   public void initializePhysicalSensors(URI sensorURI)
   {
      robotPoseBuffer = new RobotPoseBuffer(sensorSuitePacketCommunicator, 10000, timestampProvider);
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);
      
      this.depthDataProcessor = new DepthDataProcessor(sensorSuitePacketCommunicator,lidarDataFilter);
      RosMainNode rosMainNode = new RosMainNode(sensorURI, "darpaRoboticsChallange/networkProcessor");

      RosNativeNetworkProcessor rosNativeNetworkProcessor;
      if (RosNativeNetworkProcessor.hasNativeLibrary())
      {
         rosNativeNetworkProcessor = RosNativeNetworkProcessor.getInstance(sensorURI.toString());
         rosNativeNetworkProcessor.connect();
      }
      else
      {
         rosNativeNetworkProcessor = null;
      }

      DRCRobotCameraParameters cameraParamaters = sensorInformation.getCameraParameters(0);

      new RosCameraReceiver(cameraParamaters, robotPoseBuffer, rosMainNode, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider,null, sensorURI);

      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(cameraParamaters, rosMainNode, sensorSuitePacketCommunicator, null);
      sensorSuitePacketCommunicator.attachListener(CameraInformationPacket.class, cameraInfoServer);

      new IbeoPointCloudDataReceiver(rosMainNode, robotPoseBuffer, sensorSuitePacketCommunicator, sdfFullRobotModel, sensorInformation.getPointCloudParameters(ValkyrieSensorInformation.IBEO_ID), lidarDataFilter);
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosMainNode.execute();
   }

   @Override
   public PacketCommunicator getProcessedSensorsCommunicator()
   {
      return sensorSuitePacketCommunicator;
   }
}
