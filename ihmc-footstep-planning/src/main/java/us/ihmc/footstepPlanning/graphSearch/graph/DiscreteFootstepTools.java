package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.commonWalkingControlModules.polygonWiggling.StepConstraintPolygonTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.SideDependentList;

public class DiscreteFootstepTools
{
   /**
    * Computes a step-to-world RigidBodyTransform from the step's x, y and yaw. This transform
    * will always have no z translation, pitch and roll.
    * @param step
    * @param stepToWorldTransformToPack
    */
   public static void getStepTransform(DiscreteFootstep step, RigidBodyTransform stepToWorldTransformToPack)
   {
      getStepTransform(step.getX(), step.getY(), step.getYaw(), stepToWorldTransformToPack);
   }

   /**
    * Computes a step-to-world RigidBodyTransform from the step's x, y and yaw. This transform
    * will always have no z translation, pitch and roll.
    * @param step
    * @param stepToWorldTransformToPack
    */
   public static void getStepTransform(double x, double y, double yaw, RigidBodyTransform stepToWorldTransformToPack)
   {
      stepToWorldTransformToPack.setRotationYawAndZeroTranslation(yaw);
      stepToWorldTransformToPack.getTranslation().set(x, y, 0.0);
   }

   /**
    * Computes a step-to-world RigidBodyTransform which transforms from step frame to world frame
    * by applying snapTransform to the given step's transform
    *
    * @param step
    * @param snapTransform pre-snap to post-snap transform
    * @param transformToPack
    */
   public static void getSnappedStepTransform(DiscreteFootstep step, RigidBodyTransformReadOnly snapTransform, RigidBodyTransform transformToPack)
   {
      getStepTransform(step, transformToPack);
      snapTransform.transform(transformToPack);
   }

   public static Point3DReadOnly getStepPositionInWorld(DiscreteFootstep step, RigidBodyTransform snapTransform)
   {
      Point3D nodeInWorld = new Point3D();
      getStepPositionInWorld(step, nodeInWorld, snapTransform);

      return nodeInWorld;
   }

   public static void getStepPositionInWorld(DiscreteFootstep step, Point3DBasics stepPositionToSet, RigidBodyTransform snapTransform)
   {
      stepPositionToSet.set(step.getX(), step.getY(), 0.0);
      snapTransform.transform(stepPositionToSet);
   }

   public static Pose3DReadOnly getStepPoseInWorld(DiscreteFootstep step, RigidBodyTransform snapTransform)
   {
      Pose3D stepPoseInWorld = new Pose3D();
      getStepPoseInWorld(step, stepPoseInWorld, snapTransform);

      return stepPoseInWorld;
   }

   public static void getStepPoseInWorld(DiscreteFootstep step, Pose3DBasics stepPoseToSet, RigidBodyTransform snapTransform)
   {
      RigidBodyTransform snappedTransform = new RigidBodyTransform();
      getSnappedStepTransform(step, snapTransform, snappedTransform);
      stepPoseToSet.set(snappedTransform);
   }

   /**
    * Computes the foot polygon in world frame that corresponds to the give footstep step
    *
    * @param step
    * @param footPolygonInSoleFrame
    * @param footPolygonToPack
    */
   public static void getFootPolygon(DiscreteFootstep step, ConvexPolygon2DReadOnly footPolygonInSoleFrame, ConvexPolygon2D footPolygonToPack)
   {
      getFootPolygon(step.getX(), step.getY(), step.getYaw(), footPolygonInSoleFrame, footPolygonToPack);
   }

   /**
    * Computes the foot polygon in world frame that corresponds to the give footstep step
    *
    * @param step
    * @param footPolygonInSoleFrame
    * @param footPolygonToPack
    */
   public static void getFootPolygon(double stepX, double stepY, double stepYaw, ConvexPolygon2DReadOnly footPolygonInSoleFrame, ConvexPolygon2D footPolygonToPack)
   {
      footPolygonToPack.set(footPolygonInSoleFrame);

      RigidBodyTransform footstepTransform = new RigidBodyTransform();
      DiscreteFootstepTools.getStepTransform(stepX, stepY, stepYaw, footstepTransform);

      LogTools.warn("Snap Transform: {} {} {}", stepX, stepY, footstepTransform);

      footPolygonToPack.applyTransform(footstepTransform);
   }

   public static double getSnappedStepHeight(DiscreteFootstep step, RigidBodyTransform snapTransform)
   {
      return snapTransform.getRotation().getM20() * step.getX() + snapTransform.getRotation().getM21() * step.getY() + snapTransform.getTranslationZ();
   }

   public static LatticePoint interpolate(LatticePoint pointA, LatticePoint pointB, double alpha)
   {
      double x = EuclidCoreTools.interpolate(pointA.getX(), pointB.getX(), alpha);
      double y = EuclidCoreTools.interpolate(pointA.getY(), pointB.getY(), alpha);
      double yaw = AngleTools.interpolateAngle(pointA.getYaw(), pointB.getYaw(), alpha);
      return new LatticePoint(x, y, yaw);
   }

   public static double computeDistanceBetweenFootPolygons(DiscreteFootstep stepA,
                                                           DiscreteFootstep stepB,
                                                           SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygonsInSoleFrame)
   {
      ConvexPolygon2D footPolygonA = new ConvexPolygon2D();
      ConvexPolygon2D footPolygonB = new ConvexPolygon2D();

      getFootPolygon(stepA, footPolygonsInSoleFrame.get(stepA.getRobotSide()), footPolygonA);
      getFootPolygon(stepB, footPolygonsInSoleFrame.get(stepB.getRobotSide()), footPolygonB);

      if (StepConstraintPolygonTools.arePolygonsIntersecting(footPolygonA, footPolygonB))
      {
         return 0.0;
      }

      return StepConstraintPolygonTools.distanceBetweenPolygons(footPolygonA, footPolygonB);
   }

   public static DiscreteFootstep constructStepInPreviousStepFrame(double stepLength, double stepWidth, double stepYaw, DiscreteFootstep node)
   {
      Vector2D footstepTranslation = new Vector2D(stepLength, stepWidth);
      AxisAngle footstepRotation = new AxisAngle(node.getYaw(), 0.0, 0.0);
      footstepRotation.transform(footstepTranslation);

      return new DiscreteFootstep(node.getX() + footstepTranslation.getX(),
                              node.getY() + footstepTranslation.getY(),
                              stepYaw + node.getYaw(),
                                  node.getRobotSide().getOppositeSide());
   }
}
