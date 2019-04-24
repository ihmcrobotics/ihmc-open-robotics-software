package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.ValveLocationPacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.ros2.Ros2Node;

public class SearchForValveBehavior extends AbstractBehavior
{
   private Pose3D valveTransformToWorld;
   private double valveRadius;
   private boolean recievedNewValveLocation = false;

   protected final ConcurrentListeningQueue<ValveLocationPacket> valveLocationQueue = new ConcurrentListeningQueue<ValveLocationPacket>(10);

   public SearchForValveBehavior(String robotName, Ros2Node ros2Node)
   {
      super(robotName, "SearchForSpehereFar", ros2Node);
      createBehaviorInputSubscriber(ValveLocationPacket.class, valveLocationQueue::put);
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("Searching For The Valve");
   }

   @Override
   public void doControl()
   {
      if (valveLocationQueue.isNewPacketAvailable())
      {
         recievedValveLocation(valveLocationQueue.getLatestPacket());
      }
   }

   @Override
   public boolean isDone(double timeinState)
   {
      return recievedNewValveLocation;
   }

   @Override
   public void onBehaviorExited()
   {
      recievedNewValveLocation = false;
   }

   public Pose3D getLocation()
   {
      return valveTransformToWorld;
   }

   public double getValveRadius()
   {
      return valveRadius;
   }

   private void recievedValveLocation(ValveLocationPacket valveLocationPacket)
   {
      publishTextToSpeech("Recieved Valve Location From UI");
      valveTransformToWorld = valveLocationPacket.getValvePoseInWorld();

      valveRadius = valveLocationPacket.getValveRadius();
      recievedNewValveLocation = true;

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

}
