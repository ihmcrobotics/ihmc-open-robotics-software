package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.LocalVideoPacket;
import us.ihmc.communication.packets.sensing.CameraInformationPacket;
import us.ihmc.communication.packets.sensing.IntrinsicCameraParametersPacket;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;

/**
 * 
 *  Generate simulated camera data and camera info packet from SCS, we use only left eye.
 */
public class SCSCameraDataReceiver extends CameraDataReceiver implements PacketConsumer<LocalVideoPacket>
{
   private final boolean DEBUG = false;
   SCSCameraInfoReceiver scsCameraInfoReceiver;

   public SCSCameraDataReceiver(RobotPoseBuffer robotPoseBuffer, PacketCommunicator scsSensorsCommunicator,
         KryoLocalPacketCommunicator outgoingSensorDataCommunicator, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      super(robotPoseBuffer, outgoingSensorDataCommunicator, ppsTimestampOffsetProvider);
      
      scsSensorsCommunicator.attachListener(LocalVideoPacket.class, this);

      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;

      scsCameraInfoReceiver = new SCSCameraInfoReceiver(outgoingSensorDataCommunicator, logger);
      outgoingSensorDataCommunicator.attachListener(CameraInformationPacket.class, scsCameraInfoReceiver);
   }

   public void receivedPacket(LocalVideoPacket object)
   {
      if (DEBUG)
      {
         System.out.println(getClass().getName() + ": received local video packet!");
      }
      updateLeftEyeImage(object.getImage(), object.getTimeStamp(), object.getFieldOfView());
      scsCameraInfoReceiver.setIntrinsicPacket(object);
   }
}

class SCSCameraInfoReceiver implements CameraInfoReceiver
{
   private IntrinsicCameraParametersPacket leftParamPacket = null;
   private final PacketCommunicator packetCommunicator;
   private double lastFov = Double.NaN;
   private int numPacket = 0;
   private final int numInitialPacketsToIgnore = 5;
   private final CameraLogger logger;

   public SCSCameraInfoReceiver(PacketCommunicator packetCommunicator, CameraLogger logger)
   {
      this.packetCommunicator = packetCommunicator;
      this.logger = logger;
   }

   @Override
   public void processInformationRequest(CameraInformationPacket object)
   {
      if (object.intrinsic)
         sendIntrinsic(object.cameraId);
   }

   @Override
   public void sendIntrinsic(int cameraId)
   {
      new Thread()
      {
         @Override
         public void run()
         {
            while (leftParamPacket == null || numPacket < numInitialPacketsToIgnore)
            {
               //System.out.println("waiting for intrinsic information (ignoring first "+numInitialPacketsToIgnore+" packets)...");
               try
               {
                  Thread.sleep(1000);
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }
            }

            if (logger != null)
               logger.log(leftParamPacket.param);

            packetCommunicator.send(leftParamPacket);
            System.out.println("SCSCameraInfoReceiver:send SCS Intrinsic Parameter Packet(packet " + numPacket + ")");

         }
      }.start();
   }

   public void setIntrinsicPacket(LocalVideoPacket videoObject)
   {
      numPacket++;
      if (videoObject.getFieldOfView() != lastFov)
      {
         System.err.println("SCSCameraInfoReceiver:SCS FoV changed (packet " + numPacket + "):" + lastFov + " -> " + videoObject.getFieldOfView());
         lastFov = videoObject.getFieldOfView();
      }
      BufferedImage img = videoObject.getImage();
      double f = videoObject.getImage().getWidth() / 2 / Math.tan(videoObject.getFieldOfView() / 2);
      leftParamPacket = new IntrinsicCameraParametersPacket(f, f, 0, (img.getWidth() - 1) / 2f, (img.getHeight() - 1) / 2f, img.getWidth(), img.getHeight());

      if (logger != null)
      {
         logger.log(img, videoObject.timeStamp);
      }
   }

   public void receivedPacket(CameraInformationPacket object)
   {
      processInformationRequest(object);
   }
}
