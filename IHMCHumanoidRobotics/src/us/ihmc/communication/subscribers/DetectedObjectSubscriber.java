package us.ihmc.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.DetectedObjectPacket;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class DetectedObjectSubscriber implements PacketConsumer<DetectedObjectPacket>, Runnable
{
   private int id;
   private RigidBodyTransform pose;
   private ConcurrentLinkedQueue<DetectedObjectPacket> incomingDetectedMocapObject = new ConcurrentLinkedQueue<>();


   private ArrayList<DetectedObjectListener> listOfListeners = new ArrayList<DetectedObjectListener>();

   public DetectedObjectSubscriber()
   {
      Thread t = new Thread(this);
      t.start();
   }

   public void registerListener(DetectedObjectListener listener)
   {
      listOfListeners.add(listener);
   }

   public int getObjectId()
   {
      return id;
   }

   public RigidBodyTransform getObjectPose()
   {
      return pose;
   }

   @Override
   public void receivedPacket(DetectedObjectPacket packet)
   {
      incomingDetectedMocapObject.add(packet);
   }

   @Override
   public void run()
   {
      while (true)
      {
         DetectedObjectPacket detectedObject = incomingDetectedMocapObject.poll();
         if (detectedObject != null)
         {
            this.id = detectedObject.id;
            this.pose = detectedObject.pose;

            for (DetectedObjectListener listener : listOfListeners)
            {
               listener.updatePose(detectedObject.pose, detectedObject.id);
            }
         }

         ThreadTools.sleep(1);
      }
   }

}
