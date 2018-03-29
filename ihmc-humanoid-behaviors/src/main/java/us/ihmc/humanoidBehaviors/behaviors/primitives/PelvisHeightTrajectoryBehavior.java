package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisHeightTrajectoryBehavior extends AbstractBehavior
{
   private final YoBoolean hasInputBeenSet = new YoBoolean("hasInputBeenSet" + behaviorName, registry);
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
   private final YoBoolean trajectoryTimeElapsed = new YoBoolean(getName() + "TrajectoryTimeElapsed", registry);
   private PelvisHeightTrajectoryMessage outgoingPelvisHeightTrajectoryMessage;

   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;

   public PelvisHeightTrajectoryBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      super(outgoingCommunicationBridge);
      startTime = new YoDouble(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(getName() + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      this.yoTime = yoTime;
   }

   public void setInput(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      this.outgoingPelvisHeightTrajectoryMessage = pelvisHeightTrajectoryMessage;
         System.out.println("Pelvis height " + pelvisHeightTrajectoryMessage.getEuclideanTrajectory().taskspaceTrajectoryPoints.getLast().getPosition().getZ());
      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() &&  hasInputBeenSet())
      {
         sendMessageToController();
      }
   }

   private void sendMessageToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {      
         outgoingPelvisHeightTrajectoryMessage.setDestination(PacketDestination.UI);  
         sendPacketToController(outgoingPelvisHeightTrajectoryMessage);
         sendPacket(outgoingPelvisHeightTrajectoryMessage);
         packetHasBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingPelvisHeightTrajectoryMessage.getEuclideanTrajectory().taskspaceTrajectoryPoints.getLast().time);
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);
      
      hasInputBeenSet.set(false);
      trajectoryTimeElapsed.set(false);
      
      hasBeenInitialized.set(true);
   }

   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
      hasInputBeenSet.set(false);
      trajectoryTimeElapsed.set(false);
      outgoingPelvisHeightTrajectoryMessage = null;

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
      
      isPaused.set(false);
      isAborted.set(false);
   }



   @Override
   public boolean isDone()
   {
      if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
         trajectoryTimeElapsed.set(false);
      else
         trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());

      return trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue();
   }

   

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
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
