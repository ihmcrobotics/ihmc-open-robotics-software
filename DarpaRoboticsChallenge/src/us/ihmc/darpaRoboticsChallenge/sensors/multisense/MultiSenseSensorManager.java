package us.ihmc.darpaRoboticsChallenge.sensors.multisense;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ArmCalibrationHelper;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraLogger;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.LidarFilter;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.RosLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   private RosCameraReceiver cameraReceiver;
   
   private final DRCRobotSensorInformation sensorInformation;
   private final RobotPoseBuffer sharedRobotPoseBuffer;
   private final RosMainNode rosMainNode;
   private final DRCNetworkProcessorNetworkingManager networkingManager;
   private final SDFFullRobotModel sharedFullRobotModel;
   private final ObjectCommunicator fieldComputerClient;
   private final RosNativeNetworkProcessor rosNativeNetworkProcessor;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final LidarFilter lidarDataFilter;
   
   private static final double lidarCRC = -.0010908f;
   private final boolean USE_ROS_FOR_SENSOR_TRANSFORMS = true;
   
   private final String sensorURI;

   private DRCRobotCameraParameters cameraParamaters;
   private DRCRobotLidarParameters lidarParamaters;

   public MultiSenseSensorManager(DRCRobotSensorInformation sensorInformation, RobotPoseBuffer sharedRobotPoseBuffer, RosMainNode rosMainNode,
         DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sharedFullRobotModel, ObjectCommunicator fieldComputerClient,
         RosNativeNetworkProcessor rosNativeNetworkProcessor, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, LidarFilter lidarDataFilter,
         String sensorURI)
   {
      cameraParamaters = sensorInformation.getCameraParameters(0);
      lidarParamaters = sensorInformation.getLidarParameters(0);
      this.sensorInformation = sensorInformation;          
      this.sharedRobotPoseBuffer = sharedRobotPoseBuffer;                
      this.rosMainNode = rosMainNode;                              
      this.networkingManager = networkingManager;          
      this.sharedFullRobotModel = sharedFullRobotModel;               
      this.fieldComputerClient = fieldComputerClient;               
      this.rosNativeNetworkProcessor = rosNativeNetworkProcessor;  
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.lidarDataFilter = lidarDataFilter;           
      this.sensorURI = sensorURI;
      registerCameraReceivers();
      registerLidarReceivers();
      MultiSenseParamaterSetter.initialize(rosMainNode);
      setMultiseSenseParams(lidarParamaters.getLidarSpindleVelocity());
                                    
   }
   
   private void setMultiseSenseParams(double lidarSpindleVelocity)
   {
      MultiSenseParamaterSetter.setMultisenseResolution(rosMainNode);
      
      if (RosNativeNetworkProcessor.hasNativeLibrary())
      {
         MultiSenseParamaterSetter.setupNativeROSCommunicator(rosNativeNetworkProcessor, lidarSpindleVelocity);
      }
      else
      {
         MultiSenseParamaterSetter.setupMultisenseSpindleSpeedPublisher(rosMainNode, lidarSpindleVelocity);
      }
   }
   
   private void registerLidarReceivers()
   {
      new RosLidarDataReceiver(rosMainNode, sharedRobotPoseBuffer, networkingManager, sharedFullRobotModel, sensorInformation.getLidarParameters(0), fieldComputerClient,
            rosNativeNetworkProcessor, ppsTimestampOffsetProvider, lidarDataFilter, USE_ROS_FOR_SENSOR_TRANSFORMS, lidarCRC, sensorURI);
   }
   
   private void registerCameraReceivers()
   {
      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
      cameraReceiver = new RosCameraReceiver(cameraParamaters, sharedRobotPoseBuffer, cameraParamaters.getVideoSettings(), rosMainNode, networkingManager,
            ppsTimestampOffsetProvider,logger, sensorURI);

      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(sensorInformation.getCameraParameters(0), rosMainNode,
            networkingManager.getControllerStateHandler(),logger);
      
      networkingManager.getControllerCommandHandler().setIntrinsicServer(cameraInfoServer);
   }

   public void registerCameraListener(ArmCalibrationHelper armCalibrationHelper)
   {
      cameraReceiver.registerCameraListener(armCalibrationHelper);
      
   }
}
