package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.DetectedObjectPacket;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.thread.ThreadTools;

public class DetectedObjectSubscriber implements PacketConsumer<DetectedObjectPacket>
{
   private AtomicInteger id = new AtomicInteger();
   private AtomicReference<RigidBodyTransform> pose = new AtomicReference<RigidBodyTransform>(null);
   private ConcurrentLinkedQueue<DetectedObjectPacket> incomingDetectedMocapObject = new ConcurrentLinkedQueue<>();

   private final ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(getClass().getName());
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(threadFactory);

   private ArrayList<DetectedObjectListener> listOfListeners = new ArrayList<>();

   public DetectedObjectSubscriber()
   {
   }

   public void registerListener(DetectedObjectListener listener)
   {
      listOfListeners.add(listener);
   }

   public int getObjectId()
   {
      return id.get();
   }

   public RigidBodyTransform getObjectPose()
   {
      return pose.get();
   }

   @Override
   public void receivedPacket(DetectedObjectPacket packet)
   {
      incomingDetectedMocapObject.add(packet);
      executorService.execute(createCallListenersTask());
   }

   private Runnable createCallListenersTask()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               callListeners();
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      };
   }

   private void callListeners()
   {
      DetectedObjectPacket detectedObject = incomingDetectedMocapObject.poll();
      if (detectedObject != null)
      {
         id.set(detectedObject.id);
         pose.set(detectedObject.pose);

         for (DetectedObjectListener listener : listOfListeners)
         {
            listener.updatePose(detectedObject.pose, detectedObject.id);
         }
      }
   }

   public void shutdown()
   {
      executorService.shutdownNow();
   }
}
