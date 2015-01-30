package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.FingerStateBehavior;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.Task;

public class FingerStateTask implements Task
{
   private static final boolean DEBUG = false;

   private final FingerStatePacket fingerStatePacket;
   private final FingerStateBehavior fingerStateBehavior;

   public FingerStateTask(RobotSide robotSide, FingerState fingerState, FingerStateBehavior fingerStateBehavior)
   {
      this.fingerStatePacket = new FingerStatePacket(robotSide, fingerState);

      this.fingerStateBehavior = fingerStateBehavior;
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("started a finger Task");
      fingerStateBehavior.initialize();
      fingerStateBehavior.setInput(fingerStatePacket);
   }

   @Override
   public void doAction()
   {
      fingerStateBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("finished a finger Task");
      fingerStateBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return fingerStateBehavior.isDone();
   }
}
