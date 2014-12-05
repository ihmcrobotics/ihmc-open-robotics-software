package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.Task;

public class ArmPoseTask implements Task
{
   private static final boolean DEBUG = false;
   private final HandPosePacket handPosePacket;
   private final HandPoseBehavior handPoseBehavior;
   
   public ArmPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      this.handPoseBehavior = handPoseBehavior;
      
      handPosePacket = new HandPosePacket(robotSide, trajectoryTime, desiredArmJointAngles);
      
   }

   @Override
   public void doTransitionIntoAction()
   {
      handPoseBehavior.initialize();
      handPoseBehavior.setInput(handPosePacket);
   }

   @Override
   public void doAction()
   {
      handPoseBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("Finished " + handPosePacket.robotSide.getCamelCaseNameForMiddleOfExpression() + " arm pose");
      handPoseBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return handPoseBehavior.isDone();
   }
   
}
