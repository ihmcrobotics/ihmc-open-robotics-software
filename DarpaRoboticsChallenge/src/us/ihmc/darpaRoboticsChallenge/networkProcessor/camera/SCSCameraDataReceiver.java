package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorControllerStateHandler;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.graphics3DAdapter.camera.CameraInformationPacket;
import us.ihmc.graphics3DAdapter.camera.IntrinsicCameraParametersPacket;
import us.ihmc.graphics3DAdapter.camera.LocalVideoPacket;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * 
 *  Generate simulated camera data and camera info packet from SCS, we use only left eye.
 *  @ref: GazeboCameraDataReceiver, MultiSenseCameraInfoReciever
 */
public class SCSCameraDataReceiver extends CameraDataReceiver implements ObjectConsumer<LocalVideoPacket>
{
   SCSCameraInfoReceiver scsCameraInfoReceiver;
   
   public SCSCameraDataReceiver(RobotPoseBuffer robotPoseBuffer, DRCRobotCameraParamaters drcRobotCameraParamaters, ObjectCommunicator scsCommunicator, DRCNetworkProcessorNetworkingManager networkingManager, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      super(robotPoseBuffer, drcRobotCameraParamaters.getVideoSettings(), networkingManager, ppsTimestampOffsetProvider);
      scsCommunicator.attachListener(LocalVideoPacket.class, this);
      scsCameraInfoReceiver = new SCSCameraInfoReceiver(networkingManager.getControllerStateHandler());
      networkingManager.getControllerCommandHandler().setIntrinsicServer(scsCameraInfoReceiver);
   }

   public void consumeObject(LocalVideoPacket object)
   {
      updateLeftEyeImage(object.getImage(), object.getTimeStamp(), object.getFieldOfView());
      scsCameraInfoReceiver.setIntrinsicPacket(object);
    }
}

class SCSCameraInfoReceiver implements CameraInfoReceiver
{
   private IntrinsicCameraParametersPacket leftParamPacket=null;
   private DRCNetworkProcessorControllerStateHandler stateHandler;
   private double lastFov=Double.NaN;
   private int numPacket=0;
   private final int numInitialPacketsToIgnore=5;
   
   public SCSCameraInfoReceiver(DRCNetworkProcessorControllerStateHandler stateHandler)
   {
      this.stateHandler = stateHandler;
   }
   
   @Override
   public void processInformationRequest(CameraInformationPacket object)
   {
      if(object.intrinsic)
         sendIntrinsic(object.cameraId);
   }
      
   @Override
   public void sendIntrinsic(int cameraId)
   {
      new Thread(){
         @Override
         public void run()
         {
            while(leftParamPacket==null || numPacket<numInitialPacketsToIgnore)
            {
               System.out.println("waiting for intrinsic information (ignoring first "+numInitialPacketsToIgnore+" packets)...");
               try
               {
                  Thread.sleep(100);
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }
            }
            stateHandler.sendSerializableObject(leftParamPacket);
            System.out.println("SCSCameraInfoReceiver:send SCS Intrinsic Parameter Packet(packet "+numPacket+")");
         
         }
      }.start();
   }
   
   public void setIntrinsicPacket(LocalVideoPacket videoObject)
   {
      numPacket++;
      if(videoObject.getFieldOfView()!=lastFov)
      {
         System.err.println("SCSCameraInfoReceiver:SCS FoV changed (packet "+ numPacket +"):" + lastFov + " -> " + videoObject.getFieldOfView());
         lastFov=videoObject.getFieldOfView();
      }
      BufferedImage img = videoObject.getImage();
      double f = videoObject.getImage().getWidth()/2 / Math.tan(videoObject.getFieldOfView()/2);
      leftParamPacket = new IntrinsicCameraParametersPacket(f, f, 0, (img.getWidth()-1)/2f,(img.getHeight()-1)/2f , img.getWidth(), img.getHeight());
   }


};
