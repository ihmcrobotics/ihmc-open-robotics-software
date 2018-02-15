package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootTrajectoryBehavior extends AbstractBehavior
{
   private FootTrajectoryMessage outgoingFootTrajectoryMessage;

   private final YoBoolean hasPacketBeenSent;
   private final YoDouble yoTime;
   private final YoDouble startTime;
   private final YoDouble trajectoryTime;
   private final YoBoolean trajectoryTimeElapsed;
   private final YoBoolean doubleSupport;

   public FootTrajectoryBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime, YoBoolean yoDoubleSupport)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      this.doubleSupport = yoDoubleSupport;

      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new YoBoolean(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new YoDouble(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new YoBoolean(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
   }

   public void setInput(FootTrajectoryMessage message)
   {
      this.outgoingFootTrajectoryMessage = message;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingFootTrajectoryMessage != null))
      {
         sendFootPosePacketToController();
      }

      if (hasPacketBeenSent.getBooleanValue() && !isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         if (Double.isNaN(startTime.getDoubleValue()) && !doubleSupport.getBooleanValue())
         {
            startTime.set(yoTime.getDoubleValue());
         }
      }
   }

   private void sendFootPosePacketToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         outgoingFootTrajectoryMessage.setDestination(PacketDestination.UI);
         sendPacketToController(outgoingFootTrajectoryMessage);
         hasPacketBeenSent.set(true);
         trajectoryTime.set(outgoingFootTrajectoryMessage.getSe3Trajectory().getTrajectoryTime());
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      hasPacketBeenSent.set(false);
      
      hasBeenInitialized.set(true);
      
      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public void onBehaviorExited()
   {
      hasPacketBeenSent.set(false);
      outgoingFootTrajectoryMessage = null;

      isPaused.set(false);
      isAborted.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
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

   @Override
   public void onBehaviorResumed()
   {
   }

   

 
}
