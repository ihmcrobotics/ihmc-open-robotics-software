package us.ihmc.darpaRoboticsChallenge.sensors.multisense;

import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPointCloudParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ArmCalibrationHelper;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraLogger;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.MultisenseParameterPacket;
import us.ihmc.darpaRoboticsChallenge.ros.ROSNativeTransformTools;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotPosePublisher;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   private RosCameraReceiver cameraReceiver;

   private final RobotPoseBuffer sharedRobotPoseBuffer;
   private final RosMainNode rosMainNode;
   private final DRCNetworkProcessorNetworkingManager networkingManager;
   private final RosNativeNetworkProcessor rosNativeNetworkProcessor;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   private static final double lidarCRC = -.0010908f;

   private final String sensorURI;

   private final DRCRobotCameraParameters cameraParamaters;
   private final DRCRobotLidarParameters lidarParamaters;

   private final DepthDataProcessor depthDataProcessor;

   private final ROSNativeTransformTools rosTransformProvider;

   private final DRCRobotPointCloudParameters stereoParamaters;

   private ParameterTree params;

   private MultisenseLidarDataReceiver multisenseLidarDataReceiver;

   public MultiSenseSensorManager(DepthDataProcessor depthDataProcessor, ROSNativeTransformTools rosTransformProvider, RobotPoseBuffer sharedRobotPoseBuffer,
         RosMainNode rosMainNode, DRCNetworkProcessorNetworkingManager networkingManager, RosNativeNetworkProcessor rosNativeNetworkProcessor,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider, String sensorURI, DRCRobotCameraParameters cameraParamaters,
         DRCRobotLidarParameters lidarParamaters, DRCRobotPointCloudParameters stereoParamaters)
   {
      this.depthDataProcessor = depthDataProcessor;
      this.rosTransformProvider = rosTransformProvider;
      this.lidarParamaters = lidarParamaters;
      this.stereoParamaters = stereoParamaters;
      this.sharedRobotPoseBuffer = sharedRobotPoseBuffer;
      this.cameraParamaters = cameraParamaters;
      this.rosMainNode = rosMainNode;
      this.networkingManager = networkingManager;
      this.rosNativeNetworkProcessor = rosNativeNetworkProcessor;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorURI = sensorURI;
      registerCameraReceivers();
      registerLidarReceivers();
      MultiSenseParamaterSetter.initialize(rosMainNode);
      setMultiseSenseParams(lidarParamaters.getLidarSpindleVelocity());
      networkingManager.getControllerCommandHandler().setMultiSenseSensorManager(this);

   }

   public void initializeParameterListeners()
   {

      System.out.println("initialise parameteres--------------------------------------------------------------------------------");

      rosMainNode.attachParameterListener("/multisense/motor_speed", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            System.out.println("new motor speed received");
            send();
         }
      });
      rosMainNode.attachParameterListener("/multisense/gain", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            System.out.println("new gain received");
            send();
         }
      });

      rosMainNode.attachParameterListener("/multisense/led_duty_cycle", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            System.out.println("new dutyCycle received");
            send();
         }
      });
      rosMainNode.attachParameterListener("/multisense/lighting", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            System.out.println("new led received");
            send();
         }
      });
      rosMainNode.attachParameterListener("/multisense/flash", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            System.out.println("new flash received");
            send();
         }
      });

      rosMainNode.attachParameterListener("/multisense/auto_exposure", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            System.out.println("new auto expo received");
            send();
         }
      });

      rosMainNode.attachParameterListener("/multisense/motor_speed", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            System.out.println("new motor speed received");
            send();
         }
      });
      
      rosMainNode.attachParameterListener("/multisense/auto_white_balance", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
            System.out.println("new auto white balance received");
            send();
         }
      });

   }

   public void handleMultisenseParameters(MultisenseParameterPacket object)
   {
      if (object.isFromUI())
      {
         if (rosMainNode.isStarted())
         {
            params = rosMainNode.getParameters();
            send();

         }
      }
      else
         MultiSenseParamaterSetter.setMultisenseParameters(object);

   }

   private void send()
   {
      if (params == null)
      {
         System.out.println("params are null");
         return;
      }

      networkingManager.getControllerStateHandler().sendSerializableObject(
            new MultisenseParameterPacket(false, params.getDouble("/multisense/gain"), params.getDouble("/multisense/motor_speed"), params
                  .getDouble("/multisense/led_duty_cycle"), params.getString("/multisense/resolution"), params.getBoolean("/multisense/lighting"), params
                  .getBoolean("/multisense/flash"), params.getBoolean("multisense/auto_exposure"), params.getBoolean("multisense/auto_white_balance")));
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
      this.multisenseLidarDataReceiver = new MultisenseLidarDataReceiver(depthDataProcessor, rosTransformProvider, ppsTimestampOffsetProvider, sharedRobotPoseBuffer, rosMainNode, lidarParamaters);
   }

   private void registerCameraReceivers()
   {
      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
      cameraReceiver = new RosCameraReceiver(cameraParamaters, sharedRobotPoseBuffer, cameraParamaters.getVideoSettings(), rosMainNode, networkingManager,
            ppsTimestampOffsetProvider, logger, sensorURI);

      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(cameraParamaters, rosMainNode, networkingManager.getControllerStateHandler(), logger);

      networkingManager.getControllerCommandHandler().setIntrinsicServer(cameraInfoServer);
   }

   public void registerCameraListener(ArmCalibrationHelper armCalibrationHelper)
   {
      cameraReceiver.registerCameraListener(armCalibrationHelper);

   }

   public void setRobotPosePublisher(RosRobotPosePublisher robotPosePublisher)
   {
      multisenseLidarDataReceiver.setRobotPosePublisher(robotPosePublisher);
      cameraReceiver.setRobotPosePublisher(robotPosePublisher);
   }
}
