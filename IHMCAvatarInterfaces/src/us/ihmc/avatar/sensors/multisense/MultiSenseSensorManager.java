package us.ihmc.avatar.sensors.multisense;

import java.net.URI;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.CameraLogger;
import us.ihmc.ihmcPerception.camera.RosCameraCompressedImageReceiver;
import us.ihmc.ihmcPerception.camera.VideoPacketHandler;
import us.ihmc.ihmcPerception.depthData.PointCloudDataReceiver;
import us.ihmc.ihmcPerception.depthData.PointCloudSource;
import us.ihmc.ihmcPerception.depthData.RosPointCloudReceiver;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   public static boolean LOG_PRIMARY_CAMERA_IMAGES = false;
   
   private CameraDataReceiver cameraReceiver;

   private final FullHumanoidRobotModelFactory fullRobotModelFactory;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final RosMainNode rosMainNode;
   private final PacketCommunicator packetCommunicator;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;


   private final URI sensorURI;

   private final DRCRobotCameraParameters cameraParameters;
   private final DRCRobotLidarParameters lidarParameters;

   private final PointCloudDataReceiver pointCloudDataReceiver;
   private MultiSenseParamaterSetter multiSenseParameterSetter;

   public MultiSenseSensorManager(FullHumanoidRobotModelFactory sdfFullRobotModelFactory, PointCloudDataReceiver pointCloudDataReceiver, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         RosMainNode rosMainNode, PacketCommunicator sensorSuitePacketCommunicator, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, URI sensorURI, DRCRobotCameraParameters cameraParameters,
         DRCRobotLidarParameters lidarParameters, DRCRobotPointCloudParameters stereoParameters, boolean setROSParameters)
   {
      this.fullRobotModelFactory = sdfFullRobotModelFactory;
      this.pointCloudDataReceiver = pointCloudDataReceiver;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.lidarParameters = lidarParameters;
      this.cameraParameters = cameraParameters;
      this.rosMainNode = rosMainNode;
      this.packetCommunicator = sensorSuitePacketCommunicator;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorURI = sensorURI;
      registerCameraReceivers();
      registerLidarReceivers(sdfFullRobotModelFactory);
      if(setROSParameters)
      {
         multiSenseParameterSetter = new MultiSenseParamaterSetter(rosMainNode, sensorSuitePacketCommunicator);
         setMultiseSenseParams(lidarParameters.getLidarSpindleVelocity());
      }
      else
      {
         multiSenseParameterSetter = null;
      }
   }

   public void initializeParameterListeners()
   {

      System.out.println("initialise parameteres--------------------------------------------------------------------------------");
      if(multiSenseParameterSetter != null)
      {
         multiSenseParameterSetter.initializeParameterListeners();
//         multiSenseParameterSetter.setLidarSpindleSpeed(lidarParameters.getLidarSpindleVelocity());
      }
   }

   private void setMultiseSenseParams(double lidarSpindleVelocity)
   {
      if(multiSenseParameterSetter != null)
      {
         multiSenseParameterSetter.setMultisenseResolution(rosMainNode);

         multiSenseParameterSetter.setupNativeROSCommunicator(lidarSpindleVelocity);
      }
   }

   private void registerLidarReceivers(FullHumanoidRobotModelFactory sdfFullRobotModelFactory)
   {
//      new RosPointCloudReceiver(lidarParameters.getSensorNameInSdf(), lidarParameters.getRosTopic(), rosMainNode, ReferenceFrame.getWorldFrame(), pointCloudDataReceiver,PointCloudSource.NEARSCAN);
//      new RosPointCloudReceiver(lidarParameters.getSensorNameInSdf(), lidarParameters.getGroundCloudTopic(), rosMainNode, ReferenceFrame.getWorldFrame(), pointCloudDataReceiver,PointCloudSource.QUADTREE);

      FullRobotModel fullRobotModel = sdfFullRobotModelFactory.createFullRobotModel();

      new RosPointCloudReceiver(lidarParameters.getRosTopic(), rosMainNode, ReferenceFrame.getWorldFrame(), fullRobotModel.getLidarBaseFrame(lidarParameters.getSensorNameInSdf()),
            pointCloudDataReceiver, PointCloudSource.NEARSCAN);

      new RosPointCloudReceiver(lidarParameters.getGroundCloudTopic(), rosMainNode, ReferenceFrame.getWorldFrame(), fullRobotModel.getLidarBaseFrame(lidarParameters.getSensorNameInSdf()),
            pointCloudDataReceiver, PointCloudSource.QUADTREE);
   }

   private void registerCameraReceivers()
   {
      CameraLogger logger = LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
      cameraReceiver = new CameraDataReceiver(fullRobotModelFactory, cameraParameters.getPoseFrameForSdf(), robotConfigurationDataBuffer, new VideoPacketHandler(packetCommunicator),
            ppsTimestampOffsetProvider);

      new RosCameraCompressedImageReceiver(cameraParameters, rosMainNode, logger, cameraReceiver);

      cameraReceiver.start();
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      cameraReceiver.registerCameraListener(drcStereoListener);

   }
}
