package us.ihmc.valkyrie.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.packets.sensing.CameraInformationPacket;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataFilter;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.darpaRoboticsChallenge.sensors.ibeo.IbeoPointCloudDataReceiver;
import us.ihmc.ros.jni.wrapper.RosNativeNetworkProcessor;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.valkyrie.paramaters.ValkyrieSensorInformation;

public class ValkyrieSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final URI rosCoreURI;
   private final DRCRobotSensorInformation sensorInformation;

   public ValkyrieSensorSuiteManager(URI rosCoreURI, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation)
   {
      this.rosCoreURI = rosCoreURI;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
   }

   public void initializeSimulatedSensors(LocalObjectCommunicator scsCommunicator, ObjectCommunicator fieldCommunicator, RobotPoseBuffer robotPoseBuffer,
                                          AbstractNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sdfFullRobotModel, DepthDataFilter lidarDataFilter, URI sensorURI, DRCRobotModel robotModel)
   {
      new SCSCameraDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, ppsTimestampOffsetProvider);

      new SCSPointCloudDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, sdfFullRobotModel, sensorInformation, scsCommunicator,
           ppsTimestampOffsetProvider, lidarDataFilter);

   }

   public void initializePhysicalSensors(RobotPoseBuffer robotPoseBuffer, AbstractNetworkProcessorNetworkingManager networkingManager,
                                         SDFFullRobotModel sdfFullRobotModel, ObjectCommunicator objectCommunicator, DepthDataFilter lidarDataFilter, URI sensorURI, DRCRobotModel robotModel)
   {
      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, "darpaRoboticsChallange/networkProcessor");

      RosNativeNetworkProcessor rosNativeNetworkProcessor;
      if (RosNativeNetworkProcessor.hasNativeLibrary())
      {
         rosNativeNetworkProcessor = RosNativeNetworkProcessor.getInstance(rosCoreURI.toString());
         rosNativeNetworkProcessor.connect();
      }
      else
      {
         rosNativeNetworkProcessor = null;
      }

      DRCRobotCameraParameters cameraParamaters = sensorInformation.getCameraParameters(0);

      new RosCameraReceiver(cameraParamaters, robotPoseBuffer, rosMainNode, networkingManager, ppsTimestampOffsetProvider,null, sensorURI);

      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(cameraParamaters, rosMainNode, networkingManager.getControllerStateHandler(),null);
      networkingManager.getControllerCommandHandler().attachListener(CameraInformationPacket.class, cameraInfoServer);

      new IbeoPointCloudDataReceiver(rosMainNode, robotPoseBuffer, networkingManager, sdfFullRobotModel, sensorInformation.getPointCloudParameters(ValkyrieSensorInformation.IBEO_ID), lidarDataFilter);

      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);

      rosMainNode.execute();
   }
}
