package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class AimPalmNormalAtPointTask extends BehaviorTask
{
   private final HandPoseBehavior handPoseBehavior;

   private final RobotSide robotSide;
   private final FramePoint targetToAimAt;
   private final FullRobotModel fullRobotModel;
   private final double trajectoryTime;

   public AimPalmNormalAtPointTask(RobotSide robotSide, FramePoint targetToAimAt, FullRobotModel fullRobotModel, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior,
         double trajectoryTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;

      this.robotSide = robotSide;
      this.fullRobotModel = fullRobotModel;
      this.targetToAimAt = targetToAimAt;
      this.trajectoryTime = trajectoryTime;
   }

   @Override
   protected void setBehaviorInput()
   {
      handPoseBehavior.aimPalmNormalAtPoint(robotSide, targetToAimAt, fullRobotModel, trajectoryTime);
   }
}
