package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisTrajectoryBehavior extends AbstractBehavior
{
   private PelvisTrajectoryMessage outgoingPelvisTrajectoryMessage;

   private final YoBoolean hasPacketBeenSent;
   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;
   private final YoBoolean trajectoryTimeElapsed;

   public PelvisTrajectoryBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new YoBoolean(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
   }

   public void setInput(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      this.outgoingPelvisTrajectoryMessage = pelvisTrajectoryMessage;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingPelvisTrajectoryMessage != null))
      {
         sendPelvisPosePacketToController();
      }
   }

   private void sendPelvisPosePacketToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         outgoingPelvisTrajectoryMessage.setDestination(PacketDestination.UI.ordinal());
         sendPacket(outgoingPelvisTrajectoryMessage);
         sendPacketToController(outgoingPelvisTrajectoryMessage);
         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingPelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime());
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      hasPacketBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);
      
      hasBeenInitialized.set(true);
   }

   @Override
   public void onBehaviorExited()
   {
      hasPacketBeenSent.set(false);
      outgoingPelvisTrajectoryMessage = null;

      isPaused.set(false);
      isAborted.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
   }



   @Override //TODO: Not currently implemented for this behavior
   public void onBehaviorResumed()
   {
      isPaused.set(false);
      startTime.set(yoTime.getDoubleValue());
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

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

  
  
}
