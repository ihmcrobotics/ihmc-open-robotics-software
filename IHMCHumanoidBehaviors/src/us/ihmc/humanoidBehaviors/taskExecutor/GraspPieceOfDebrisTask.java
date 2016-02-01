package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspPieceOfDebrisBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class GraspPieceOfDebrisTask extends BehaviorTask
{
   private final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior;
   private final RigidBodyTransform debrisTransform;
   private final Point3d graspPosition;
   private final Vector3d graspVector;
   private final RobotSide robotSide;

   public GraspPieceOfDebrisTask(GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior, RigidBodyTransform debrisTransform, Point3d graspPosition,
         Vector3d graspVector, RobotSide robotSide, DoubleYoVariable yoTime)
   {
      super(graspPieceOfDebrisBehavior, yoTime);
      this.graspPieceOfDebrisBehavior = graspPieceOfDebrisBehavior;
      this.debrisTransform = debrisTransform;
      this.graspPosition = graspPosition;
      this.graspVector = graspVector;
      this.robotSide = robotSide;
   }

   @Override
   protected void setBehaviorInput()
   {
      graspPieceOfDebrisBehavior.setInput(debrisTransform, graspPosition, graspVector, robotSide);
   }

}
