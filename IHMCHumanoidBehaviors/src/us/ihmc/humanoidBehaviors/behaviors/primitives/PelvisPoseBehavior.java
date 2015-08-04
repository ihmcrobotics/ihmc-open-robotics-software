package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.FormattingTools;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class PelvisPoseBehavior extends BehaviorInterface
{
   private PelvisPosePacket outgoingPelvisPosePacket;

   private final BooleanYoVariable hasPacketBeenSent;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final BooleanYoVariable trajectoryTimeElapsed;

   public PelvisPoseBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      String behaviorNameFirstLowerCase = FormattingTools.lowerCaseFirstLetter(getName());
      hasPacketBeenSent = new BooleanYoVariable(behaviorNameFirstLowerCase + "HasPacketBeenSent", registry);
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new BooleanYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
   }

   public void setInput(PelvisPosePacket pelvisPosePacket)
   {
      this.outgoingPelvisPosePacket = pelvisPosePacket;
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSent.getBooleanValue() && (outgoingPelvisPosePacket != null))
      {
         sendPelvisPosePacketToController();
      }
   }

   private void sendPelvisPosePacketToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         outgoingPelvisPosePacket.setDestination(PacketDestination.UI);
         sendPacketToNetworkProcessor(outgoingPelvisPosePacket);
         sendPacketToController(outgoingPelvisPosePacket);
         hasPacketBeenSent.set(true);
         startTime.set(yoTime.getDoubleValue());
         trajectoryTime.set(outgoingPelvisPosePacket.getTrajectoryTime());
      }
   }

   @Override
   public void initialize()
   {
      hasPacketBeenSent.set(false);

      isPaused.set(false);
      isStopped.set(false);
      
      hasBeenInitialized.set(true);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      hasPacketBeenSent.set(false);
      outgoingPelvisPosePacket = null;

      isPaused.set(false);
      isStopped.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
   }

   @Override //TODO: Not currently implemented for this behavior
   public void stop()
   {
      isStopped.set(true);
   }

   @Override //TODO: Not currently implemented for this behavior
   public void pause()
   {
      isPaused.set(true);
   }

   @Override //TODO: Not currently implemented for this behavior
   public void resume()
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
      return outgoingPelvisPosePacket != null;
   }
}
