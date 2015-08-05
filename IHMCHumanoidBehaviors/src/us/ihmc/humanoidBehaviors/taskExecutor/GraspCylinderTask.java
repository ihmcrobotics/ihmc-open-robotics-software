package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspCylinderTask extends BehaviorTask
{
   private final HandPoseBehavior handPoseBehavior;

   private final RobotSide robotSide;

   private final FramePoint graspTarget;
   private final FrameVector graspCylinderLongAxis;

   private final FullRobotModel fullRobotModel;
   private final double trajectoryTime;

   public GraspCylinderTask(RobotSide robotSide, FramePoint graspTarget, FrameVector graspCylinderLongAxis, FullRobotModel fullRobotModel,
         DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;

      this.robotSide = robotSide;
      this.fullRobotModel = fullRobotModel;
      this.graspTarget = graspTarget;
      this.graspCylinderLongAxis = graspCylinderLongAxis;
      this.trajectoryTime = trajectoryTime;
   }

   @Override
   protected void setBehaviorInput()
   {
      handPoseBehavior.orientAndMoveHandToGraspCylinder(robotSide, graspCylinderLongAxis, graspTarget, fullRobotModel, trajectoryTime);
   }
}
