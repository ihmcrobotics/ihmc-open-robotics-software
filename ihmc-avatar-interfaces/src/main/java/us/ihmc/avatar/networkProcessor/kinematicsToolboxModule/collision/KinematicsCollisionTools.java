package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.EuclidShapeCollisionTools;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Tools for evaluating collisions. It is mainly used via
 * {@link KinematicsCollidable#evaluateCollision(KinematicsCollidable)}.
 * 
 * @author Sylvain Bertrand
 */
public class KinematicsCollisionTools
{
   public static Box3D changeFrame(Box3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Box3D transformed = new Box3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Capsule3D changeFrame(Capsule3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Capsule3D transformed = new Capsule3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Cylinder3D changeFrame(Cylinder3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Cylinder3D transformed = new Cylinder3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Ellipsoid3D changeFrame(Ellipsoid3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Ellipsoid3D transformed = new Ellipsoid3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static PointShape3D changeFrame(PointShape3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      PointShape3D transformed = new PointShape3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Ramp3D changeFrame(Ramp3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Ramp3D transformed = new Ramp3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Sphere3D changeFrame(Sphere3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Sphere3D transformed = new Sphere3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   static interface Shape3DCollisionEvaluator<A extends Shape3DReadOnly, B extends Shape3DReadOnly>
   {
      void evaluateCollision(A shapeA, B shapeB, EuclidShape3DCollisionResultBasics resultToPack);
   }

   static interface FrameShape3DCollisionEvaluator<A extends Shape3DReadOnly, B extends Shape3DReadOnly>
   {
      void evaluateCollision(A shapeA, ReferenceFrame frameA, B shapeB, ReferenceFrame frameB, KinematicsCollisionResult resultToPack);
   }

   static interface FrameChanger<A extends Shape3DReadOnly>
   {
      A changeFrame(A original, ReferenceFrame from, ReferenceFrame to);
   }

   static final FrameChanger<Box3DReadOnly> box3DFrameChanger = KinematicsCollisionTools::changeFrame;
   static final FrameChanger<Capsule3DReadOnly> capsule3DFrameChanger = KinematicsCollisionTools::changeFrame;
   static final FrameChanger<Cylinder3DReadOnly> cylinder3DFrameChanger = KinematicsCollisionTools::changeFrame;
   static final FrameChanger<Ellipsoid3DReadOnly> ellipsoid3DFrameChanger = KinematicsCollisionTools::changeFrame;
   static final FrameChanger<PointShape3DReadOnly> pointShape3DFrameChanger = KinematicsCollisionTools::changeFrame;
   static final FrameChanger<Ramp3DReadOnly> ramp3DFrameChanger = KinematicsCollisionTools::changeFrame;
   static final FrameChanger<Sphere3DReadOnly> sphere3DFrameChanger = KinematicsCollisionTools::changeFrame;

   @SuppressWarnings("unchecked")
   static <A extends Shape3DReadOnly> FrameChanger<A> getFrameChanger(A shape)
   {
      if (shape instanceof Box3DReadOnly)
         return (FrameChanger<A>) box3DFrameChanger;
      else if (shape instanceof Capsule3DReadOnly)
         return (FrameChanger<A>) capsule3DFrameChanger;
      else if (shape instanceof Cylinder3DReadOnly)
         return (FrameChanger<A>) cylinder3DFrameChanger;
      else if (shape instanceof Ellipsoid3DReadOnly)
         return (FrameChanger<A>) ellipsoid3DFrameChanger;
      else if (shape instanceof PointShape3DReadOnly)
         return (FrameChanger<A>) pointShape3DFrameChanger;
      else if (shape instanceof Ramp3DReadOnly)
         return (FrameChanger<A>) ramp3DFrameChanger;
      else if (shape instanceof Sphere3DReadOnly)
         return (FrameChanger<A>) sphere3DFrameChanger;
      else
         return null;
   }

   static final Shape3DCollisionEvaluator<Capsule3DReadOnly, Capsule3DReadOnly> capsule3DToCapsule3DEvaluator = EuclidShapeCollisionTools::evaluateCapsule3DCapsule3DCollision;
   static final Shape3DCollisionEvaluator<PointShape3DReadOnly, Box3DReadOnly> pointShape3DToBox3DEvaluator = EuclidShapeCollisionTools::evaluatePointShape3DBox3DCollision;
   static final Shape3DCollisionEvaluator<PointShape3DReadOnly, Capsule3DReadOnly> pointShape3DToCapsule3DEvaluator = EuclidShapeCollisionTools::evaluatePointShape3DCapsule3DCollision;
   static final Shape3DCollisionEvaluator<PointShape3DReadOnly, Cylinder3DReadOnly> pointShape3DToCylinder3DEvaluator = EuclidShapeCollisionTools::evaluatePointShape3DCylinder3DCollision;
   static final Shape3DCollisionEvaluator<PointShape3DReadOnly, Ellipsoid3DReadOnly> pointShape3DToEllipsoid3DEvaluator = EuclidShapeCollisionTools::evaluatePointShape3DEllipsoid3DCollision;
   static final Shape3DCollisionEvaluator<PointShape3DReadOnly, Ramp3DReadOnly> pointShape3DToRamp3DEvaluator = EuclidShapeCollisionTools::evaluatePointShape3DRamp3DCollision;
   static final Shape3DCollisionEvaluator<PointShape3DReadOnly, Sphere3DReadOnly> pointShape3DToSphere3DEvaluator = EuclidShapeCollisionTools::evaluatePointShape3DSphere3DCollision;
   static final Shape3DCollisionEvaluator<PointShape3DReadOnly, PointShape3DReadOnly> pointShape3DToPointShape3DEvaluator = new Shape3DCollisionEvaluator<PointShape3DReadOnly, PointShape3DReadOnly>()
   {
      @Override
      public void evaluateCollision(PointShape3DReadOnly shapeA, PointShape3DReadOnly shapeB, EuclidShape3DCollisionResultBasics resultToPack)
      {
         resultToPack.setShapeA(shapeA);
         resultToPack.setShapeB(shapeB);
         resultToPack.setShapesAreColliding(false);
         resultToPack.setSignedDistance(shapeA.distance(shapeB));
         resultToPack.getPointOnA().set(shapeA);
         resultToPack.getPointOnB().set(shapeB);
         resultToPack.getNormalOnA().sub(shapeB, shapeA);
         resultToPack.getNormalOnB().sub(shapeA, shapeB);
      }
   };

   static final Shape3DCollisionEvaluator<Sphere3DReadOnly, Box3DReadOnly> sphere3DToBox3DEvaluator = EuclidShapeCollisionTools::evaluateSphere3DBox3DCollision;
   static final Shape3DCollisionEvaluator<Sphere3DReadOnly, Capsule3DReadOnly> sphere3DToCapsule3DEvaluator = EuclidShapeCollisionTools::evaluateSphere3DCapsule3DCollision;
   static final Shape3DCollisionEvaluator<Sphere3DReadOnly, Cylinder3DReadOnly> sphere3DToCylinder3DEvaluator = EuclidShapeCollisionTools::evaluateSphere3DCylinder3DCollision;
   static final Shape3DCollisionEvaluator<Sphere3DReadOnly, Ellipsoid3DReadOnly> sphere3DToEllipsoid3DEvaluator = EuclidShapeCollisionTools::evaluateSphere3DEllipsoid3DCollision;
   static final Shape3DCollisionEvaluator<Sphere3DReadOnly, Ramp3DReadOnly> sphere3DToRamp3DEvaluator = EuclidShapeCollisionTools::evaluateSphere3DRamp3DCollision;
   static final Shape3DCollisionEvaluator<Sphere3DReadOnly, Sphere3DReadOnly> sphere3DToSphere3DEvaluator = EuclidShapeCollisionTools::evaluateSphere3DSphere3DCollision;

   static <A extends Shape3DReadOnly, B extends Shape3DReadOnly> void evaluateFrameCollision(A shapeA, ReferenceFrame frameA, B shapeB, ReferenceFrame frameB,
                                                                                             Shape3DCollisionEvaluator<A, B> collisionEvaluator,
                                                                                             FrameChanger<A> shapeAFrameChanger,
                                                                                             KinematicsCollisionResult resultToPack)
   {
      collisionEvaluator.evaluateCollision(shapeAFrameChanger.changeFrame(shapeA, frameA, frameB), shapeB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setFrameA(frameA);
      resultToPack.setFrameB(frameB);
      resultToPack.getPointOnA().setReferenceFrame(frameB);
      resultToPack.getPointOnB().setReferenceFrame(frameB);
      resultToPack.getNormalOnA().setReferenceFrame(frameB);
      resultToPack.getNormalOnB().setReferenceFrame(frameB);
   }

   public static void evaluateCapsule3DCapsule3DCollision(Capsule3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB, ReferenceFrame frameB,
                                                          KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();
      pointOnA.setIncludingFrame(frameA, shapeA.getTopCenter());
      pointOnA.changeFrame(frameB);
      double topAX = pointOnA.getX();
      double topAY = pointOnA.getY();
      double topAZ = pointOnA.getZ();
      pointOnA.setIncludingFrame(frameA, shapeA.getBottomCenter());
      pointOnA.changeFrame(frameB);
      double bottomAX = pointOnA.getX();
      double bottomAY = pointOnA.getY();
      double bottomAZ = pointOnA.getZ();
      double topBX = shapeB.getTopCenter().getX();
      double topBY = shapeB.getTopCenter().getY();
      double topBZ = shapeB.getTopCenter().getZ();
      double bottomBX = shapeB.getBottomCenter().getX();
      double bottomBY = shapeB.getBottomCenter().getY();
      double bottomBZ = shapeB.getBottomCenter().getZ();

      double distanceBetweenAxes = EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(topAX,
                                                                                               topAY,
                                                                                               topAZ,
                                                                                               bottomAX,
                                                                                               bottomAY,
                                                                                               bottomAZ,
                                                                                               topBX,
                                                                                               topBY,
                                                                                               topBZ,
                                                                                               bottomBX,
                                                                                               bottomBY,
                                                                                               bottomBZ,
                                                                                               pointOnA,
                                                                                               pointOnB);
      pointOnA.setReferenceFrame(frameB);
      pointOnB.setReferenceFrame(frameB);

      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();
      normalOnA.setReferenceFrame(frameB);
      normalOnB.setReferenceFrame(frameB);

      normalOnA.sub(pointOnB, pointOnA);
      normalOnA.scale(1.0 / distanceBetweenAxes);
      normalOnB.setAndNegate(normalOnA);

      pointOnA.scaleAdd(shapeA.getRadius(), normalOnA, pointOnA);
      pointOnB.scaleAdd(shapeB.getRadius(), normalOnB, pointOnB);

      double distance = distanceBetweenAxes - shapeA.getRadius() - shapeB.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);

      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      resultToPack.setFrameA(frameA);
      resultToPack.setFrameB(frameB);
   }

   public static void evaluatePointShape3DBox3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Box3DReadOnly shapeB, ReferenceFrame frameB,
                                                         KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DBox3DCollision(shapeA, frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }

   private static void evaluatePoint3DBox3DCollision(Point3DReadOnly point3D, ReferenceFrame pointFrame, Box3DReadOnly box3D, ReferenceFrame boxFrame,
                                                     KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();

      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(pointFrame, point3D);
      pointOnA.changeFrame(boxFrame);

      double pointInBBackupX = pointOnA.getX();
      double pointInBBackupY = pointOnA.getY();
      double pointInBBackupZ = pointOnA.getZ();

      box3D.getPose().inverseTransform(pointOnA);

      double distance = EuclidShapeTools.evaluatePoint3DBox3DCollision(pointOnA, box3D.getSize(), pointOnB, normalOnB);
      pointOnB.setReferenceFrame(boxFrame);
      normalOnB.setReferenceFrame(boxFrame);
      normalOnA.setReferenceFrame(boxFrame);

      box3D.transformToWorld(pointOnB);
      box3D.transformToWorld(normalOnB);
      pointOnA.setIncludingFrame(boxFrame, pointInBBackupX, pointInBBackupY, pointInBBackupZ);
      normalOnA.setAndNegate(normalOnB);
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setFrameA(pointFrame);
      resultToPack.setFrameB(boxFrame);
   }

   public static void evaluatePointShape3DCapsule3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB,
                                                             ReferenceFrame frameB, KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DCapsule3DCollision(shapeA, frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }

   private static void evaluatePoint3DCapsule3DCollision(Point3DReadOnly point3D, ReferenceFrame pointFrame, Capsule3DReadOnly capsule3D,
                                                         ReferenceFrame capsuleFrame, KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();
      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(pointFrame, point3D);
      pointOnA.changeFrame(capsuleFrame);

      double distance = EuclidShapeTools.evaluatePoint3DCapsule3DCollision(pointOnA,
                                                                           capsule3D.getPosition(),
                                                                           capsule3D.getAxis(),
                                                                           capsule3D.getLength(),
                                                                           capsule3D.getRadius(),
                                                                           pointOnB,
                                                                           normalOnB);
      pointOnB.setReferenceFrame(capsuleFrame);
      normalOnA.setReferenceFrame(capsuleFrame);
      normalOnB.setReferenceFrame(capsuleFrame);

      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setFrameA(pointFrame);
      resultToPack.setFrameB(capsuleFrame);
   }

   public static void evaluatePointShape3DCylinder3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Cylinder3DReadOnly shapeB,
                                                              ReferenceFrame frameB, KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DCylinder3DCollision(shapeA, frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }

   private static void evaluatePoint3DCylinder3DCollision(Point3DReadOnly point3D, ReferenceFrame pointFrame, Cylinder3DReadOnly cylinder3D,
                                                          ReferenceFrame cylinderFrame, KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();
      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(pointFrame, point3D);
      pointOnA.changeFrame(cylinderFrame);

      double distance = EuclidShapeTools.evaluatePoint3DCylinder3DCollision(pointOnA,
                                                                            cylinder3D.getPosition(),
                                                                            cylinder3D.getAxis(),
                                                                            cylinder3D.getLength(),
                                                                            cylinder3D.getRadius(),
                                                                            pointOnB,
                                                                            normalOnB);
      pointOnB.setReferenceFrame(cylinderFrame);
      normalOnA.setReferenceFrame(cylinderFrame);
      normalOnB.setReferenceFrame(cylinderFrame);

      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setFrameA(pointFrame);
      resultToPack.setFrameB(cylinderFrame);
   }

   public static void evaluatePointShape3DEllipsoid3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Ellipsoid3DReadOnly shapeB,
                                                               ReferenceFrame frameB, KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DEllipsoid3DCollision(shapeA, frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }

   private static void evaluatePoint3DEllipsoid3DCollision(Point3DReadOnly point3D, ReferenceFrame pointFrame, Ellipsoid3DReadOnly ellipsoid3D,
                                                           ReferenceFrame ellipsoidFrame, KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();
      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(pointFrame, point3D);
      pointOnA.changeFrame(ellipsoidFrame);

      double pointInBBackupX = pointOnA.getX();
      double pointInBBackupY = pointOnA.getY();
      double pointInBBackupZ = pointOnA.getZ();

      ellipsoid3D.getPose().inverseTransform(pointOnA);

      double distance = EuclidShapeTools.evaluatePoint3DEllipsoid3DCollision(pointOnA, ellipsoid3D.getRadii(), pointOnB, normalOnB);
      pointOnB.setReferenceFrame(ellipsoidFrame);
      normalOnA.setReferenceFrame(ellipsoidFrame);
      normalOnB.setReferenceFrame(ellipsoidFrame);
      ellipsoid3D.transformToWorld(pointOnB);
      ellipsoid3D.transformToWorld(normalOnB);

      pointOnA.setIncludingFrame(ellipsoidFrame, pointInBBackupX, pointInBBackupY, pointInBBackupZ);
      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setFrameA(pointFrame);
      resultToPack.setFrameB(ellipsoidFrame);
   }

   public static void evaluatePointShape3DPointShape3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, PointShape3DReadOnly shapeB,
                                                                ReferenceFrame frameB, KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();
      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(frameA, shapeA);
      pointOnA.changeFrame(frameB);
      pointOnB.setIncludingFrame(frameB, shapeB);
      normalOnA.setReferenceFrame(frameB);
      normalOnB.setReferenceFrame(frameB);
      normalOnA.sub(pointOnB, pointOnA);
      normalOnB.sub(pointOnA, pointOnB);

      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      resultToPack.setShapesAreColliding(false);
      resultToPack.setSignedDistance(pointOnA.distance(shapeB));
      resultToPack.setFrameA(frameA);
      resultToPack.setFrameB(frameB);
   }

   public static void evaluatePointShape3DRamp3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Ramp3DReadOnly shapeB, ReferenceFrame frameB,
                                                          KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DRamp3DCollision(shapeA, frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }

   private static void evaluatePoint3DRamp3DCollision(Point3DReadOnly point3D, ReferenceFrame pointFrame, Ramp3DReadOnly ramp3D, ReferenceFrame rampFrame,
                                                      KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();
      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(pointFrame, point3D);
      pointOnA.changeFrame(rampFrame);

      double pointInBBackupX = pointOnA.getX();
      double pointInBBackupY = pointOnA.getY();
      double pointInBBackupZ = pointOnA.getZ();

      ramp3D.getPose().inverseTransform(pointOnA);
      double distance = EuclidShapeTools.evaluatePoint3DRamp3DCollision(pointOnA, ramp3D.getSize(), pointOnB, normalOnB);
      pointOnB.setReferenceFrame(rampFrame);
      normalOnA.setReferenceFrame(rampFrame);
      normalOnB.setReferenceFrame(rampFrame);

      ramp3D.transformToWorld(pointOnB);
      ramp3D.transformToWorld(normalOnB);
      pointOnA.setIncludingFrame(rampFrame, pointInBBackupX, pointInBBackupY, pointInBBackupZ);
      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setFrameA(pointFrame);
      resultToPack.setFrameB(rampFrame);
   }

   public static void evaluatePointShape3DSphere3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Sphere3DReadOnly shapeB, ReferenceFrame frameB,
                                                            KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();
      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(frameA, shapeA);
      pointOnA.changeFrame(frameB);
      double distance = EuclidShapeTools.evaluatePoint3DSphere3DCollision(pointOnA, shapeB.getPosition(), shapeB.getRadius(), pointOnB, normalOnB);
      pointOnB.setReferenceFrame(frameB);
      normalOnA.setReferenceFrame(frameB);
      normalOnB.setReferenceFrame(frameB);

      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      resultToPack.setFrameA(frameA);
      resultToPack.setFrameB(frameB);
   }

   public static void evaluateSphere3DBox3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Box3DReadOnly shapeB, ReferenceFrame frameB,
                                                     KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DBox3DCollision(shapeA.getPosition(), frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setSignedDistance(distance);
      resultToPack.setShapesAreColliding(distance < 0.0);
   }

   public static void evaluateSphere3DCapsule3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB, ReferenceFrame frameB,
                                                         KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DCapsule3DCollision(shapeA.getPosition(), frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluateSphere3DCylinder3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Cylinder3DReadOnly shapeB, ReferenceFrame frameB,
                                                          KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DCylinder3DCollision(shapeA.getPosition(), frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluateSphere3DEllipsoid3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Ellipsoid3DReadOnly shapeB, ReferenceFrame frameB,
                                                           KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DEllipsoid3DCollision(shapeA.getPosition(), frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluateSphere3DRamp3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Ramp3DReadOnly shapeB, ReferenceFrame frameB,
                                                      KinematicsCollisionResult resultToPack)
   {
      evaluatePoint3DRamp3DCollision(shapeA.getPosition(), frameA, shapeB, frameB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluateSphere3DSphere3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Sphere3DReadOnly shapeB, ReferenceFrame frameB,
                                                        KinematicsCollisionResult resultToPack)
   {
      FramePoint3D pointOnA = resultToPack.getPointOnA();
      FramePoint3D pointOnB = resultToPack.getPointOnB();
      FrameVector3D normalOnA = resultToPack.getNormalOnA();
      FrameVector3D normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(frameA, shapeA.getPosition());
      pointOnA.changeFrame(frameB);
      double distance = EuclidShapeTools.evaluatePoint3DSphere3DCollision(pointOnA, shapeB.getPosition(), shapeB.getRadius(), pointOnB, normalOnB);
      distance -= shapeA.getRadius();

      pointOnB.setReferenceFrame(frameB);
      normalOnA.setReferenceFrame(frameB);
      normalOnB.setReferenceFrame(frameB);
      normalOnA.setAndNegate(normalOnB);
      pointOnA.scaleAdd(shapeA.getRadius(), normalOnA, resultToPack.getPointOnA());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      resultToPack.setFrameA(frameA);
      resultToPack.setFrameB(frameB);
   }

   public static void evaluateShape3DBox3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Box3DReadOnly shapeB, ReferenceFrame frameB,
                                                    KinematicsCollisionResult resultToPack)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         evaluatePointShape3DBox3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else if (shapeA instanceof Sphere3DReadOnly)
         evaluateSphere3DBox3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else
         evaluateShape3DBox3DCollisionEPA(shapeA, frameA, shapeB, frameB, resultToPack);
   }

   static void evaluateShape3DBox3DCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, Box3DReadOnly shapeB, ReferenceFrame frameB,
                                                KinematicsCollisionResult resultToPack)
   {
      evaluateFrameCollisionEPA(shapeA, frameA, shapeB, frameB, box3DFrameChanger, resultToPack);
   }

   public static void evaluateShape3DCapsule3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB, ReferenceFrame frameB,
                                                        KinematicsCollisionResult resultToPack)
   {
      if (shapeA instanceof Capsule3DReadOnly)
      {
         evaluateCapsule3DCapsule3DCollision(shapeB, frameB, (Capsule3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof PointShape3DReadOnly)
         evaluatePointShape3DCapsule3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else if (shapeA instanceof Sphere3DReadOnly)
         evaluateSphere3DCapsule3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else
         evaluateShape3DCapsule3DCollisionEPA(shapeA, frameA, shapeB, frameB, resultToPack);
   }

   static void evaluateShape3DCapsule3DCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB, ReferenceFrame frameB,
                                                    KinematicsCollisionResult resultToPack)
   {
      evaluateFrameCollisionEPA(shapeA, frameA, shapeB, frameB, capsule3DFrameChanger, resultToPack);
   }

   public static void evaluateShape3DCylinder3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Cylinder3DReadOnly shapeB, ReferenceFrame frameB,
                                                         KinematicsCollisionResult resultToPack)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         evaluatePointShape3DCylinder3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else if (shapeA instanceof Sphere3DReadOnly)
         evaluateSphere3DCylinder3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else
         evaluateShape3DCylinder3DCollisionEPA(shapeA, frameA, shapeB, frameB, resultToPack);
   }

   static void evaluateShape3DCylinder3DCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, Cylinder3DReadOnly shapeB, ReferenceFrame frameB,
                                                     KinematicsCollisionResult resultToPack)
   {
      evaluateFrameCollisionEPA(shapeA, frameA, shapeB, frameB, cylinder3DFrameChanger, resultToPack);
   }

   public static void evaluateShape3DEllipsoid3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Ellipsoid3DReadOnly shapeB, ReferenceFrame frameB,
                                                          KinematicsCollisionResult resultToPack)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         evaluatePointShape3DEllipsoid3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else if (shapeA instanceof Sphere3DReadOnly)
         evaluateSphere3DEllipsoid3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else
         evaluateShape3DEllipsoid3DCollisionEPA(shapeA, frameA, shapeB, frameB, resultToPack);
   }

   static void evaluateShape3DEllipsoid3DCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, Ellipsoid3DReadOnly shapeB, ReferenceFrame frameB,
                                                      KinematicsCollisionResult resultToPack)
   {
      evaluateFrameCollisionEPA(shapeA, frameA, shapeB, frameB, ellipsoid3DFrameChanger, resultToPack);
   }

   public static void evaluateShape3DPointShape3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, PointShape3DReadOnly shapeB, ReferenceFrame frameB,
                                                           KinematicsCollisionResult resultToPack)
   {
      if (shapeA instanceof Box3DReadOnly)
      {
         evaluatePointShape3DBox3DCollision(shapeB, frameB, (Box3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Capsule3DReadOnly)
      {
         evaluatePointShape3DCapsule3DCollision(shapeB, frameB, (Capsule3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Cylinder3DReadOnly)
      {
         evaluatePointShape3DCylinder3DCollision(shapeB, frameB, (Cylinder3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Ellipsoid3DReadOnly)
      {
         evaluatePointShape3DEllipsoid3DCollision(shapeB, frameB, (Ellipsoid3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof PointShape3DReadOnly)
      {
         evaluatePointShape3DPointShape3DCollision(shapeB, frameB, (PointShape3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Ramp3DReadOnly)
      {
         evaluatePointShape3DRamp3DCollision(shapeB, frameB, (Ramp3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Sphere3DReadOnly)
      {
         evaluatePointShape3DSphere3DCollision(shapeB, frameB, (Sphere3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else
      {
         evaluateShape3DPointShape3DCollisionEPA(shapeA, frameA, shapeB, frameB, resultToPack);
      }
   }

   static void evaluateShape3DPointShape3DCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, PointShape3DReadOnly shapeB, ReferenceFrame frameB,
                                                       KinematicsCollisionResult resultToPack)
   {
      evaluateFrameCollisionEPA(shapeA, frameA, shapeB, frameB, pointShape3DFrameChanger, resultToPack);
   }

   public static void evaluateShape3DRamp3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Ramp3DReadOnly shapeB, ReferenceFrame frameB,
                                                     KinematicsCollisionResult resultToPack)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         evaluatePointShape3DRamp3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else if (shapeA instanceof Sphere3DReadOnly)
         evaluateSphere3DRamp3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      else
         evaluateShape3DRamp3DCollisionEPA(shapeA, frameA, shapeB, frameB, resultToPack);
   }

   static void evaluateShape3DRamp3DCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, Ramp3DReadOnly shapeB, ReferenceFrame frameB,
                                                 KinematicsCollisionResult resultToPack)
   {
      evaluateFrameCollisionEPA(shapeA, frameA, shapeB, frameB, ramp3DFrameChanger, resultToPack);
   }

   public static void evaluateShape3DSphere3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Sphere3DReadOnly shapeB, ReferenceFrame frameB,
                                                       KinematicsCollisionResult resultToPack)
   {
      if (shapeA instanceof Box3DReadOnly)
      {
         evaluateSphere3DBox3DCollision(shapeB, frameB, (Box3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Capsule3DReadOnly)
      {
         evaluateSphere3DCapsule3DCollision(shapeB, frameB, (Capsule3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Cylinder3DReadOnly)
      {
         evaluateSphere3DCylinder3DCollision(shapeB, frameB, (Cylinder3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Ellipsoid3DReadOnly)
      {
         evaluateSphere3DEllipsoid3DCollision(shapeB, frameB, (Ellipsoid3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof PointShape3DReadOnly)
      {
         evaluatePointShape3DSphere3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB, resultToPack);
      }
      else if (shapeA instanceof Ramp3DReadOnly)
      {
         evaluateSphere3DRamp3DCollision(shapeB, frameB, (Ramp3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof Sphere3DReadOnly)
      {
         evaluateSphere3DSphere3DCollision(shapeB, frameB, (Sphere3DReadOnly) shapeA, frameA, resultToPack);
         resultToPack.swapShapes();
      }
      else
      {
         evaluateShape3DSphere3DCollisionEPA(shapeA, frameA, shapeB, frameB, resultToPack);
      }
   }

   static void evaluateShape3DSphere3DCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, Sphere3DReadOnly shapeB, ReferenceFrame frameB,
                                                   KinematicsCollisionResult resultToPack)
   {
      evaluateFrameCollisionEPA(shapeA, frameA, shapeB, frameB, sphere3DFrameChanger, resultToPack);
   }

   static <B extends Shape3DReadOnly> void evaluateFrameCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, B shapeB, ReferenceFrame frameB,
                                                                     FrameChanger<B> shapeBFrameChanger, KinematicsCollisionResult resultToPack)
   {
      resultToPack.setToZero();
      boolean colliding = new ExpandingPolytopeAlgorithm().evaluateCollision(shapeA, shapeBFrameChanger.changeFrame(shapeB, frameB, frameA), resultToPack);
      resultToPack.setShapesAreColliding(colliding); // TODO Bug in Euclid.
      resultToPack.setShapeB(shapeB);
      resultToPack.setFrameA(frameA);
      resultToPack.setFrameB(frameB);
      resultToPack.getPointOnA().setReferenceFrame(frameA);
      resultToPack.getPointOnB().setReferenceFrame(frameA);
      resultToPack.getNormalOnA().setReferenceFrame(frameA);
      resultToPack.getNormalOnB().setReferenceFrame(frameA);
   }

   public static void evaluateShape3DShape3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Shape3DReadOnly shapeB, ReferenceFrame frameB,
                                                      KinematicsCollisionResult resultToPack)
   {
      if (shapeB instanceof Box3DReadOnly)
         evaluateShape3DBox3DCollision(shapeA, frameA, (Box3DReadOnly) shapeB, frameB, resultToPack);
      else if (shapeB instanceof Capsule3DReadOnly)
         evaluateShape3DCapsule3DCollision(shapeA, frameA, (Capsule3DReadOnly) shapeB, frameB, resultToPack);
      else if (shapeB instanceof Cylinder3DReadOnly)
         evaluateShape3DCylinder3DCollision(shapeA, frameA, (Cylinder3DReadOnly) shapeB, frameB, resultToPack);
      else if (shapeB instanceof Ellipsoid3DReadOnly)
         evaluateShape3DEllipsoid3DCollision(shapeA, frameA, (Ellipsoid3DReadOnly) shapeB, frameB, resultToPack);
      else if (shapeB instanceof PointShape3DReadOnly)
         evaluateShape3DPointShape3DCollision(shapeA, frameA, (PointShape3DReadOnly) shapeB, frameB, resultToPack);
      else if (shapeB instanceof Ramp3DReadOnly)
         evaluateShape3DRamp3DCollision(shapeA, frameA, (Ramp3DReadOnly) shapeB, frameB, resultToPack);
      else if (shapeB instanceof Sphere3DReadOnly)
         evaluateShape3DSphere3DCollision(shapeA, frameA, (Sphere3DReadOnly) shapeB, frameB, resultToPack);
      else
         evaluateShape3DShape3DCollisionEPA(shapeA, frameA, shapeB, frameB, resultToPack);

   }

   static void evaluateShape3DShape3DCollisionEPA(Shape3DReadOnly shapeA, ReferenceFrame frameA, Shape3DReadOnly shapeB, ReferenceFrame frameB,
                                                  KinematicsCollisionResult resultToPack)
   {
      RigidBodyTransform transformToA = frameB.getTransformToDesiredFrame(frameA);
      Vector3D localSupportDirection = new Vector3D();

      SupportingVertexHolder localShapeB = (supportDirection, supportingVertexToPack) ->
      {
         transformToA.inverseTransform(supportDirection, localSupportDirection);
         boolean success = shapeB.getSupportingVertex(localSupportDirection, supportingVertexToPack);
         if (success)
            transformToA.transform(supportingVertexToPack);
         return success;
      };
      new ExpandingPolytopeAlgorithm().evaluateCollision(shapeA, localShapeB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      resultToPack.setFrameA(frameA);
      resultToPack.setFrameB(frameB);
      resultToPack.getPointOnA().setReferenceFrame(frameA);
      resultToPack.getPointOnB().setReferenceFrame(frameA);
      resultToPack.getNormalOnA().setReferenceFrame(frameA);
      resultToPack.getNormalOnB().setReferenceFrame(frameA);
   }
}
