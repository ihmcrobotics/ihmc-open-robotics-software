package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class FingerStateBehavior extends BehaviorInterface
{
   private FingerStatePacket outgoingFingerStatePacket;
   private final BooleanYoVariable hasInputBeenSet;
   private final BooleanYoVariable hasPacketBeenSet;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime; // hardcoded, to be determined
   private final BooleanYoVariable trajectoryTimeElapsed;

   public FingerStateBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridgeInterface, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridgeInterface);
      this.yoTime = yoTime;

      hasInputBeenSet = new BooleanYoVariable(getName() + "hasInputBeenSet", registry);
      hasPacketBeenSet = new BooleanYoVariable(getName() + "hasPacketBeenSet", registry);

      startTime = new DoubleYoVariable(getName() + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(getName() + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);

      trajectoryTimeElapsed = new BooleanYoVariable(getName() + "TrajectoryTimeElapsed", registry);
   }

   public void setInput(FingerStatePacket fingerStatePacket)
   {
      this.outgoingFingerStatePacket = fingerStatePacket;
      hasInputBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      if (!hasPacketBeenSet.getBooleanValue() && outgoingFingerStatePacket != null)
         sendFingerStateToController();
   }

   private void sendFingerStateToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         outgoingFingerStatePacket.setDestination(PacketDestination.CONTROLLER);

         sendPacketToController(outgoingFingerStatePacket);
         sendPacketToNetworkProcessor(outgoingFingerStatePacket);
         hasPacketBeenSet.set(true);
         startTime.set(yoTime.getDoubleValue());
      }
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
      for (RobotSide robotSide : RobotSide.values())
      {
         FingerStatePacket stopFingerStatePacket = new FingerStatePacket(robotSide, FingerState.STOP);
         stopFingerStatePacket.setDestination(PacketDestination.UI);
         sendPacketToController(stopFingerStatePacket);
         sendPacketToNetworkProcessor(stopFingerStatePacket);
      }
      isStopped.set(true);
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   public void pause()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         FingerStatePacket stopFingerStatePacket = new FingerStatePacket(robotSide, FingerState.STOP);
         stopFingerStatePacket.setDestination(PacketDestination.UI);
         sendPacketToController(stopFingerStatePacket);
         sendPacketToNetworkProcessor(stopFingerStatePacket);
      }
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
      hasPacketBeenSet.set(false);
      if (hasInputBeenSet())
      {
         sendFingerStateToController();
      }
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
   public void initialize()
   {
      if (hasInputBeenSet())
      {
         SysoutTool.println("Re-Initializing");
      }
      hasInputBeenSet.set(false);
      hasPacketBeenSet.set(false);
      outgoingFingerStatePacket = null;
      isPaused.set(false);
      isStopped.set(false);
      trajectoryTime.set(1.0); //hardCoded to be determined
      
      trajectoryTimeElapsed.set(false);
   }

   @Override
   public void finalize()
   {
      hasInputBeenSet.set(false);
      hasPacketBeenSet.set(false);
      outgoingFingerStatePacket = null;
      isPaused.set(false);
      isStopped.set(false);

      trajectoryTime.set(Double.NaN);
      startTime.set(Double.NaN);
      trajectoryTimeElapsed.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
