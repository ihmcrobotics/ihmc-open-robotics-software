package us.ihmc.humanoidBehaviors.behaviors.primitives;

import javax.vecmath.Point3d;

import us.ihmc.communication.packets.LookAtStatus;
import us.ihmc.communication.packets.sensing.LookAtPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;

public class LookAtBehavior extends BehaviorInterface
{
   private LookAtPacket packetToSend;
   private int lookAtPacketIndex = 0;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable inputsHaveBeenSupplied;
   private final ConcurrentListeningQueue<LookAtPacket> inputListeningQueue = new ConcurrentListeningQueue<LookAtPacket>();
   private final ConcurrentListeningQueue<LookAtStatus> statusListeningQueue = new ConcurrentListeningQueue<LookAtStatus>();

   public LookAtBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
      isDone = new BooleanYoVariable(behaviorName + "_isDone", registry);
      inputsHaveBeenSupplied = new BooleanYoVariable(behaviorName + "_inputsHaveBeenSupplied", registry);
   }

   @Override
   public void doControl()
   {
      if (inputListeningQueue.isNewPacketAvailable())
      {
         packetToSend = inputListeningQueue.getNewestPacket();
         lookAtPacketIndex = packetToSend.getIndex();
         inputsHaveBeenSupplied.set(true);
      }

      if (packetToSend != null)
      {
         sendPacketToController(packetToSend);
         packetToSend = null;
      }

      if (statusListeningQueue.isNewPacketAvailable())
      {
         LookAtStatus lookAtResult = statusListeningQueue.getNewestPacket();
         if (lookAtResult.getIndex() == lookAtPacketIndex)
         {
            isDone.set(lookAtResult.isFinished());
         }
      }
   }

   public void setLookAtLocation(Point3d pointToLookAt)
   {
      lookAtPacketIndex++;
      packetToSend = new LookAtPacket(pointToLookAt, lookAtPacketIndex);
      inputsHaveBeenSupplied.set(true);
   }

   public void reset()
   {
      packetToSend = null;
      inputsHaveBeenSupplied.set(false);
      isDone.set(false);
      inputListeningQueue.clear();
      statusListeningQueue.clear();
   }

   public boolean isLooking()
   {
      return hasInputBeenSet() && !isDone();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {

   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {

   }

   @Override
   public void stop()
   {

   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void pause()
   {

   }

   @Override
   public void resume()
   {

   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void finalize()
   {
      reset();
   }

   @Override
   public void initialize()
   {
      reset();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return inputsHaveBeenSupplied.getBooleanValue();
   }
}
