package us.ihmc.humanoidBehaviors.behaviors.primitives;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ChestTrajectoryBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = false;
   
   private ChestTrajectoryMessage outgoingChestTrajectoryMessage;

   private final BooleanYoVariable hasPacketBeenSent;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final BooleanYoVariable trajectoryTimeHasElapsed;

   public ChestTrajectoryBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      hasPacketBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeHasElapsed = new BooleanYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeHasElapsed", registry);
   }

   public void setInput(ChestTrajectoryMessage chestOrientationPacket)
   {
      this.outgoingChestTrajectoryMessage = chestOrientationPacket;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingChestTrajectoryMessage != null))
      {
         sendChestPoseToController();
      }
   }

   private void sendChestPoseToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         outgoingChestTrajectoryMessage.setDestination(PacketDestination.UI);
         sendPacket(outgoingChestTrajectoryMessage);
         sendPacketToController(outgoingChestTrajectoryMessage);
         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingChestTrajectoryMessage.getTrajectoryTime());
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
      outgoingChestTrajectoryMessage = null;

      isPaused.set(false);
      isAborted.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
   }




   @Override
   public boolean isDone()
   {
      boolean startTimeUndefined = Double.isNaN(startTime.getDoubleValue());
      boolean trajectoryTimeUndefined = Double.isNaN(trajectoryTime.getDoubleValue());
      double trajectoryTimeElapsed = yoTime.getDoubleValue() - startTime.getDoubleValue();

      if (DEBUG)
      {
         PrintTools.debug(this, "StartTimeUndefined: " + startTimeUndefined + ".  TrajectoryTimeUndefined: " + trajectoryTimeUndefined);
         PrintTools.debug(this, "TrajectoryTimeElapsed: " + trajectoryTimeElapsed);
      }

      if ( startTimeUndefined || trajectoryTimeUndefined )
         trajectoryTimeHasElapsed.set(false);
      else
         trajectoryTimeHasElapsed.set( trajectoryTimeElapsed > trajectoryTime.getDoubleValue());

      return trajectoryTimeHasElapsed.getBooleanValue() && !isPaused.getBooleanValue();
   }

   public boolean hasInputBeenSet()
   {
      return outgoingChestTrajectoryMessage != null;
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
