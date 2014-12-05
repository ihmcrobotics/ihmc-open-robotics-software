package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class ArmPoseTask implements Task
{
   private static final boolean DEBUG = false;
   private final HandPosePacket handPosePacket;
   private final HandPoseBehavior handPoseBehavior;

   private final DoubleYoVariable yoTime;
   private double behaviorDoneTime = Double.NaN;
   private final double sleepTime;

   public ArmPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      this(robotSide, desiredArmJointAngles, yoTime, handPoseBehavior, trajectoryTime, 0.0);
   }

   public ArmPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime,
         double sleepTime)
   {
      this.handPoseBehavior = handPoseBehavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;

      handPosePacket = new HandPosePacket(robotSide, trajectoryTime, desiredArmJointAngles);

   }

   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("Started arm task");
      handPoseBehavior.initialize();
      handPoseBehavior.setInput(handPosePacket);
   }

   @Override
   public void doAction()
   {
      handPoseBehavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && handPoseBehavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
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
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return handPoseBehavior.isDone() && sleepTimeAchieved;
   }

}
