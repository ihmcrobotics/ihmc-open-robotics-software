package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.RotateHandAboutAxisBehavior;
import us.ihmc.utilities.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class RotateHandAboutAxisTask extends BehaviorTask
{
   private final RobotSide robotSide;
   private final RigidBodyTransform graspedObjectTransformToWorld;
   private final Axis pinJointAxisInGraspedObjectFrame;
   private final double turnAngleRad;
   private final double rotationRateRadPerSec;
   private final boolean stopHandIfCollision;
   private final boolean controlHandOrientationAboutAxis;

   private final RotateHandAboutAxisBehavior rotateHandAboutAxisBehavior;

   public RotateHandAboutAxisTask(RobotSide robotSide, DoubleYoVariable yoTime, RotateHandAboutAxisBehavior rotateGraspedPinJointBodyBehavior,
	         RigidBodyTransform graspedObjectTransformToWorld, Axis pinJointAxisInGraspedObjectFrame, boolean controlHandOrientationAboutAxis, double turnAngleRad, double rotationRateRadPerSec, boolean stopHandIfCollision)
   {
      super(rotateGraspedPinJointBodyBehavior, yoTime);
      this.rotateHandAboutAxisBehavior = rotateGraspedPinJointBodyBehavior;
      this.robotSide = robotSide;
      this.graspedObjectTransformToWorld = new RigidBodyTransform(graspedObjectTransformToWorld); // Creating new object here prevents strange behaviors when this task is repeated
      this.pinJointAxisInGraspedObjectFrame = pinJointAxisInGraspedObjectFrame;
      this.controlHandOrientationAboutAxis = controlHandOrientationAboutAxis;
      this.turnAngleRad = turnAngleRad;
      this.rotationRateRadPerSec = rotationRateRadPerSec;
      this.stopHandIfCollision = stopHandIfCollision;
   }

   @Override
   protected void setBehaviorInput()
   {
	   rotateHandAboutAxisBehavior.setInput(robotSide, controlHandOrientationAboutAxis, pinJointAxisInGraspedObjectFrame, graspedObjectTransformToWorld, turnAngleRad, rotationRateRadPerSec, stopHandIfCollision);
   }
}
