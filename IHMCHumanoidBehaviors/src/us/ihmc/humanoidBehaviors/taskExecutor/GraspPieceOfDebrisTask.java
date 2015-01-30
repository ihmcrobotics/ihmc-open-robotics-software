package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspPieceOfDebrisBehavior;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.Task;

public class GraspPieceOfDebrisTask implements Task
{
   private final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior;
   private final RigidBodyTransform debrisTransform;
   private final Point3d graspPosition;
   private final Vector3d graspVector;
   private final RobotSide robotSide;

   public GraspPieceOfDebrisTask(GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior, RigidBodyTransform debrisTransform, Point3d graspPosition,
         Vector3d graspVector, RobotSide robotSide)
   {
      this.graspPieceOfDebrisBehavior = graspPieceOfDebrisBehavior;
      this.debrisTransform = debrisTransform;
      this.graspPosition = graspPosition;
      this.graspVector = graspVector;
      this.robotSide = robotSide;
   }

   @Override
   public void doTransitionIntoAction()
   {
      graspPieceOfDebrisBehavior.initialize();
      graspPieceOfDebrisBehavior.setGraspPose(debrisTransform, graspPosition, graspVector, robotSide);
   }

   @Override
   public void doAction()
   {
      graspPieceOfDebrisBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      graspPieceOfDebrisBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return graspPieceOfDebrisBehavior.isDone();
   }
}
