package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class ChestOrientationBehavior extends BehaviorInterface
{
   private final boolean DEBUG = false;
   
   private ChestOrientationPacket outgoingChestOrientationPacket;

   private final BooleanYoVariable hasPacketBeenSent;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final BooleanYoVariable trajectoryTimeHasElapsed;

   public ChestOrientationBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = FormattingTools.lowerCaseFirstLetter(getName());
      hasPacketBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeHasElapsed = new BooleanYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeHasElapsed", registry);
   }

   public void setInput(ChestOrientationPacket chestOrientationPacket)
   {
      this.outgoingChestOrientationPacket = chestOrientationPacket;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingChestOrientationPacket != null))
      {
         sendChestPoseToController();
      }
   }

   private void sendChestPoseToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         outgoingChestOrientationPacket.setDestination(PacketDestination.UI);
         sendPacketToNetworkProcessor(outgoingChestOrientationPacket);
         sendPacketToController(outgoingChestOrientationPacket);
         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingChestOrientationPacket.getTrajectoryTime());
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void finalize()
   {
      hasPacketBeenSent.set(false);
      outgoingChestOrientationPacket = null;

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
      boolean startTimeUndefined = Double.isNaN(startTime.getDoubleValue());
      boolean trajectoryTimeUndefined = Double.isNaN(trajectoryTime.getDoubleValue());
      double trajectoryTimeElapsed = yoTime.getDoubleValue() - startTime.getDoubleValue();
      
      SysoutTool.println("StartTimeUndefined: " + startTimeUndefined + ".  TrajectoryTimeUndefined: " + trajectoryTimeUndefined, DEBUG);
      SysoutTool.println("TrajectoryTimeElapsed: " + trajectoryTimeElapsed, DEBUG);

      if ( startTimeUndefined || trajectoryTimeUndefined )
         trajectoryTimeHasElapsed.set(false);
      else
         trajectoryTimeHasElapsed.set( trajectoryTimeElapsed > trajectoryTime.getDoubleValue());

      return trajectoryTimeHasElapsed.getBooleanValue() && !isPaused.getBooleanValue();
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
      return outgoingChestOrientationPacket != null;
   }
}
