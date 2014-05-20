package us.ihmc.darpaRoboticsChallenge.sensors.multisense;

import org.ros.exception.RemoteException;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceResponseListener;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ArmCalibrationHelper;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.GazeboLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.RobotBoundingBoxes;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import dynamic_reconfigure.Reconfigure;
import dynamic_reconfigure.ReconfigureRequest;
import dynamic_reconfigure.ReconfigureResponse;
import dynamic_reconfigure.StrParameter;

public class MultiSenseSensorManager
{
   private final RosCameraReceiver cameraReceiver;
   private final RosMainNode rosMainNode;

   public MultiSenseSensorManager(DRCRobotSensorInformation sensorInformation, RobotPoseBuffer sharedRobotPoseBuffer, RosMainNode rosMainNode,
         DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sharedFullRobotModel, ObjectCommunicator fieldComputerClient,
         RosNativeNetworkProcessor rosNativeNetworkProcessor, RobotBoundingBoxes robotBoundingBoxes, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {

      DRCRobotCameraParamaters cameraParamaters = sensorInformation.getPrimaryCameraParamaters();
      this.rosMainNode = rosMainNode;
      
      cameraReceiver = new RosCameraReceiver(cameraParamaters, sharedRobotPoseBuffer, cameraParamaters.getVideoSettings(), rosMainNode, networkingManager,
            ppsTimestampOffsetProvider);

      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(sensorInformation.getPrimaryCameraParamaters(), rosMainNode, networkingManager.getControllerStateHandler());
      networkingManager.getControllerCommandHandler().setIntrinsicServer(cameraInfoServer);

      new GazeboLidarDataReceiver(rosMainNode, sharedRobotPoseBuffer, networkingManager, sharedFullRobotModel, robotBoundingBoxes,
            sensorInformation, fieldComputerClient, rosNativeNetworkProcessor, ppsTimestampOffsetProvider);
      
      setMultisenseResolution();
   }

   private void setMultisenseResolution()
   {
      try
      {
         final RosServiceClient<ReconfigureRequest, ReconfigureResponse> multiSenseClient = new RosServiceClient<ReconfigureRequest, ReconfigureResponse>(
               Reconfigure._TYPE);
         rosMainNode.attachServiceClient("multisense_sl/set_parameters", multiSenseClient);

         Thread setupThread = new Thread()
         {
            public void run()
            {
               multiSenseClient.waitTillConnected();
               System.out.println("Setting multisense resolution to 2048x1088");
               ReconfigureRequest request = multiSenseClient.getMessage();
               StrParameter resolutionParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(StrParameter._TYPE);
               resolutionParam.setName("resolution");
               resolutionParam.setValue("2048x1088");
               request.getConfig().getStrs().add(resolutionParam);

               multiSenseClient.call(request, new ServiceResponseListener<ReconfigureResponse>()
               {

                  public void onSuccess(ReconfigureResponse response)
                  {
                     System.out.println("Set resolution to " + response.getConfig().getStrs().get(0).getValue());
                  }

                  public void onFailure(RemoteException e)
                  {
                     e.printStackTrace();
                  }
               });
            }
         };

         setupThread.start();

      }
      catch (Exception e)
      {
         System.err.println(e.getMessage());
      }
   }

   public void registerCameraListener(ArmCalibrationHelper armCalibrationHelper)
   {
      cameraReceiver.registerCameraListener(armCalibrationHelper);
      
   }
}
