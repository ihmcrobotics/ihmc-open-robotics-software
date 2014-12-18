package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseListBehavior;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseListTask implements Task
{
   private static final boolean DEBUG = false;
   private final HandPoseListPacket handPoseListPacket;
   private final HandPoseListBehavior handPoseListBehavior;

   private final DoubleYoVariable yoTime;
   private double behaviorDoneTime = Double.NaN;
   private final double sleepTime;

   public HandPoseListTask(HandPoseListPacket handPoseListPacket, HandPoseListBehavior handPoseListBehavior, DoubleYoVariable yoTime,
         double sleepTime)
   {
      this.handPoseListBehavior = handPoseListBehavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;
      this.handPoseListPacket = handPoseListPacket;
   }

   public HandPoseListTask(HandPoseListPacket handPoseListPacket, HandPoseListBehavior handPoseListBehavior, DoubleYoVariable yoTime)
   {
      this.handPoseListBehavior = handPoseListBehavior;
      this.yoTime = yoTime;
      this.sleepTime = 0.0;
      this.handPoseListPacket = handPoseListPacket;
   }
   
   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("Started arm task");
      handPoseListBehavior.initialize();
      handPoseListBehavior.setInput(handPoseListPacket);
   }

   @Override
   public void doAction()
   {
      handPoseListBehavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && handPoseListBehavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("Finished " + handPoseListPacket.robotSide.getCamelCaseNameForMiddleOfExpression() + " arm pose list");
      handPoseListBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return handPoseListBehavior.isDone() && sleepTimeAchieved;
   }

}
