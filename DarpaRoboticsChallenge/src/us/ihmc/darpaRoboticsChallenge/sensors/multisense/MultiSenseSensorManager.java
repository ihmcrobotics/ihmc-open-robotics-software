package us.ihmc.darpaRoboticsChallenge.sensors.multisense;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModelFactory;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraLogger;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudSource;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.RosPointCloudReceiver;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   private RosCameraReceiver cameraReceiver;

   private final SDFFullHumanoidRobotModelFactory fullRobotModelFactory;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final RosMainNode rosMainNode;
   private final PacketCommunicator packetCommunicator;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;


   private final URI sensorURI;

   private final DRCRobotCameraParameters cameraParamaters;
   private final DRCRobotLidarParameters lidarParamaters;

   private final PointCloudDataReceiver pointCloudDataReceiver;
   private MultiSenseParamaterSetter multiSenseParamaterSetter;

   public MultiSenseSensorManager(SDFFullHumanoidRobotModelFactory sdfFullRobotModelFactory, PointCloudDataReceiver pointCloudDataReceiver, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         RosMainNode rosMainNode, PacketCommunicator sensorSuitePacketCommunicator, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, URI sensorURI, DRCRobotCameraParameters cameraParamaters,
         DRCRobotLidarParameters lidarParamaters, DRCRobotPointCloudParameters stereoParamaters, boolean setROSParameters)
   {
      this.fullRobotModelFactory = sdfFullRobotModelFactory;
      this.pointCloudDataReceiver = pointCloudDataReceiver;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.lidarParamaters = lidarParamaters;
      this.cameraParamaters = cameraParamaters;
      this.rosMainNode = rosMainNode;
      this.packetCommunicator = sensorSuitePacketCommunicator;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorURI = sensorURI;
      registerCameraReceivers();
      registerLidarReceivers();
      if(setROSParameters)
      {
         multiSenseParamaterSetter = new MultiSenseParamaterSetter(rosMainNode, sensorSuitePacketCommunicator);
         setMultiseSenseParams(lidarParamaters.getLidarSpindleVelocity());
      }
      else
      {
         multiSenseParamaterSetter = null;
      }
   }

   public void initializeParameterListeners()
   {

      System.out.println("initialise parameteres--------------------------------------------------------------------------------");
      if(multiSenseParamaterSetter != null)
      {
         multiSenseParamaterSetter.initializeParameterListeners(); 
//         multiSenseParamaterSetter.setLidarSpindleSpeed(lidarParamaters.getLidarSpindleVelocity());
      }
   }

   private void setMultiseSenseParams(double lidarSpindleVelocity)
   {
      if(multiSenseParamaterSetter != null)
      {
         multiSenseParamaterSetter.setMultisenseResolution(rosMainNode);
   
         multiSenseParamaterSetter.setupNativeROSCommunicator(lidarSpindleVelocity);
      }
   }

   private void registerLidarReceivers()
   { 
      new RosPointCloudReceiver(lidarParamaters.getSensorNameInSdf(), lidarParamaters.getRosTopic(), rosMainNode, ReferenceFrame.getWorldFrame(), pointCloudDataReceiver,PointCloudSource.NEARSCAN);
      new RosPointCloudReceiver(lidarParamaters.getSensorNameInSdf(), lidarParamaters.getGroundCloudTopic(), rosMainNode, ReferenceFrame.getWorldFrame(), pointCloudDataReceiver,PointCloudSource.QUADTREE);

   }

   private void registerCameraReceivers()
   {
      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
      cameraReceiver = new RosCameraReceiver(fullRobotModelFactory, cameraParamaters, robotConfigurationDataBuffer, rosMainNode, packetCommunicator,
            ppsTimestampOffsetProvider, logger);

      cameraReceiver.start();
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      cameraReceiver.registerCameraListener(drcStereoListener);

   }
}
