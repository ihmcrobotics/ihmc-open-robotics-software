package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootPosePacket;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.FormattingTools;

public class FootPoseBehavior extends BehaviorInterface
{
   private FootPosePacket outgoingFootPosePacket;

   private final BooleanYoVariable hasPacketBeenSent;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final BooleanYoVariable trajectoryTimeElapsed;
   private final BooleanYoVariable doubleSupport;

   public FootPoseBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      this.doubleSupport = yoDoubleSupport;

      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new BooleanYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
   }

   public void setInput(FootPosePacket footPosePacket)
   {
      this.outgoingFootPosePacket = footPosePacket;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingFootPosePacket != null))
      {
         sendFootPosePacketToController();
      }

      if (hasPacketBeenSent.getBooleanValue() && !isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         if (Double.isNaN(startTime.getDoubleValue()) && !doubleSupport.getBooleanValue())
         {
            startTime.set(yoTime.getDoubleValue());
         }
      }
   }

   private void sendFootPosePacketToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         outgoingFootPosePacket.setDestination(PacketDestination.UI);
         sendPacketToController(outgoingFootPosePacket);
         hasPacketBeenSent.set(true);
         trajectoryTime.set(outgoingFootPosePacket.getTrajectoryTime());
      }
   }

   @Override
   public void initialize()
   {
      hasPacketBeenSent.set(false);
      
      hasBeenInitialized.set(true);
      
      isPaused.set(false);
      isStopped.set(false);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      hasPacketBeenSent.set(false);
      outgoingFootPosePacket = null;

      isPaused.set(false);
      isStopped.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
   }

   @Override
   public void stop()
   {
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
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
   public void enableActions()
   {
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
   public boolean hasInputBeenSet()
   {
      return outgoingFootPosePacket != null;
   }
}
