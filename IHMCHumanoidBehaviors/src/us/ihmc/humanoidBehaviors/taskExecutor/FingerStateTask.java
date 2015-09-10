package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class FingerStateTask extends BehaviorTask
{
   private static final boolean DEBUG = false;

   private final FingerStatePacket fingerStatePacket;
   private final FingerStateBehavior fingerStateBehavior;

   public FingerStateTask(RobotSide robotSide, FingerState fingerState, FingerStateBehavior fingerStateBehavior, DoubleYoVariable yoTime)
   {
      super(fingerStateBehavior, yoTime);
      this.fingerStatePacket = new FingerStatePacket(robotSide, fingerState);
      this.fingerStateBehavior = fingerStateBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      fingerStateBehavior.setInput(fingerStatePacket);
   }
}
