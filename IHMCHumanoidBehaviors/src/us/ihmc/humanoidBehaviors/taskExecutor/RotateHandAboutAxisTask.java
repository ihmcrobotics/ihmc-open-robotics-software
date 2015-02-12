package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.RotateHandAboutAxisBehavior;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class RotateHandAboutAxisTask extends BehaviorTask
{
   private static final boolean DEBUG = false;
   private final RobotSide robotSide;
   private final RigidBodyTransform graspedObjectTransformToWorld;
   private final Axis pinJointAxisInGraspedObjectFrame;
   private final FramePose graspHandPoseInWorld;
   private final double turnAngleRad;
   private final double trajectoryTime;

   private final RotateHandAboutAxisBehavior rotateHandAboutAxisBehavior;

   public RotateHandAboutAxisTask(RobotSide robotSide, DoubleYoVariable yoTime, RotateHandAboutAxisBehavior rotateGraspedPinJointBodyBehavior,
         RigidBodyTransform graspedObjectTransformToWorld, Axis pinJointAxisInGraspedObjectFrame, FramePose graspHandPoseInWorld, double turnAngleRad,
         double trajectoryTime)
   {
      super(rotateGraspedPinJointBodyBehavior, yoTime);
      this.rotateHandAboutAxisBehavior = rotateGraspedPinJointBodyBehavior;
      this.robotSide = robotSide;
      this.graspedObjectTransformToWorld = graspedObjectTransformToWorld;
      this.pinJointAxisInGraspedObjectFrame = pinJointAxisInGraspedObjectFrame;
      this.graspHandPoseInWorld = graspHandPoseInWorld;
      this.turnAngleRad = turnAngleRad;
      this.trajectoryTime = trajectoryTime;
   }

   @Override
   protected void setBehaviorInput()
   {
      rotateHandAboutAxisBehavior.setInput(robotSide, graspedObjectTransformToWorld, pinJointAxisInGraspedObjectFrame, graspHandPoseInWorld,
            turnAngleRad, trajectoryTime);
   }

   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      if (DEBUG)
         SysoutTool.println("Starting rotateGraspedPinJointBodyTask");
   }

   public void doTransitionOutOfAction()
   {
      super.doTransitionIntoAction();
      if (DEBUG)
         SysoutTool.println("Stopping rotateGraspedPinJointBodyTask");
   }
}
