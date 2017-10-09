package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;

public class SimpleVideoForwardingBehavior extends ImageProcessingBehavior
{
   public SimpleVideoForwardingBehavior(CommunicationBridge communicationBridge, PacketDestination packetForwardDestination)
   {
      super("SimplevVideoForwarder", communicationBridge, packetForwardDestination);
   }

   @Override
   public void processImageToSend(BufferedImage bufferedImageToPack, Point3DReadOnly cameraPositionToPack, QuaternionReadOnly cameraOrientationToPack,
         IntrinsicParameters intrinsicParametersToPack)
   {
   }


   

   @Override
   public boolean isDone()
   {
      return false;
   }



   @Override
   public void onBehaviorEntered()
   {
      for(ConcurrentListeningQueue<VideoPacket> queue : communicationBridge.getListeningNetworkQueues().get(VideoPacket.class))
      {
         queue.clear();
      }
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }

  
}
