package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.EuclidShapeCollisionTools;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

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
      default EuclidShape3DCollisionResult evaluateCollision(A shapeA, B shapeB)
      {
         EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
         evaluateCollision(shapeA, shapeB, result);
         return result;
      }

      void evaluateCollision(A shapeA, B shapeB, EuclidShape3DCollisionResultBasics resultToPack);
   }

   static interface FrameShape3DCollisionEvaluator<A extends Shape3DReadOnly, B extends Shape3DReadOnly>
   {
      KinematicsCollisionResult evaluateCollision(A shapeA, ReferenceFrame frameA, B shapeB, ReferenceFrame frameB);
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

   static <A extends Shape3DReadOnly, B extends Shape3DReadOnly> KinematicsCollisionResult evaluateFrameCollision(A shapeA, ReferenceFrame frameA, B shapeB,
                                                                                                                  ReferenceFrame frameB,
                                                                                                                  Shape3DCollisionEvaluator<A, B> collisionEvaluator,
                                                                                                                  FrameChanger<A> shapeAFrameChanger)
   {
      KinematicsCollisionResult result = new KinematicsCollisionResult();
      collisionEvaluator.evaluateCollision(shapeAFrameChanger.changeFrame(shapeA, frameA, frameB), shapeB, result);
      result.setShapeA(shapeA);
      result.setFrameA(frameA);
      result.setFrameB(frameB);
      result.getPointOnA().setReferenceFrame(frameB);
      result.getPointOnB().setReferenceFrame(frameB);
      result.getNormalOnA().setReferenceFrame(frameB);
      result.getNormalOnB().setReferenceFrame(frameB);
      return result;
   }

   public static KinematicsCollisionResult evaluateCapsule3DCapsule3DCollision(Capsule3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB,
                                                                               ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, capsule3DToCapsule3DEvaluator, capsule3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluatePointShape3DBox3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Box3DReadOnly shapeB,
                                                                              ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, pointShape3DToBox3DEvaluator, pointShape3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluatePointShape3DCapsule3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB,
                                                                                  ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, pointShape3DToCapsule3DEvaluator, pointShape3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluatePointShape3DCylinder3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA,
                                                                                   Cylinder3DReadOnly shapeB, ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, pointShape3DToCylinder3DEvaluator, pointShape3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluatePointShape3DEllipsoid3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA,
                                                                                    Ellipsoid3DReadOnly shapeB, ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, pointShape3DToEllipsoid3DEvaluator, pointShape3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluatePointShape3DPointShape3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA,
                                                                                     PointShape3DReadOnly shapeB, ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, pointShape3DToPointShape3DEvaluator, pointShape3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluatePointShape3DRamp3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Ramp3DReadOnly shapeB,
                                                                               ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, pointShape3DToRamp3DEvaluator, pointShape3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluatePointShape3DSphere3DCollision(PointShape3DReadOnly shapeA, ReferenceFrame frameA, Sphere3DReadOnly shapeB,
                                                                                 ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, pointShape3DToSphere3DEvaluator, pointShape3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateSphere3DBox3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Box3DReadOnly shapeB,
                                                                          ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, sphere3DToBox3DEvaluator, sphere3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateSphere3DCapsule3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB,
                                                                              ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, sphere3DToCapsule3DEvaluator, sphere3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateSphere3DCylinder3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Cylinder3DReadOnly shapeB,
                                                                               ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, sphere3DToCylinder3DEvaluator, sphere3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateSphere3DEllipsoid3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Ellipsoid3DReadOnly shapeB,
                                                                                ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, sphere3DToEllipsoid3DEvaluator, sphere3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateSphere3DRamp3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Ramp3DReadOnly shapeB,
                                                                           ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, sphere3DToRamp3DEvaluator, sphere3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateSphere3DSphere3DCollision(Sphere3DReadOnly shapeA, ReferenceFrame frameA, Sphere3DReadOnly shapeB,
                                                                             ReferenceFrame frameB)
   {
      return evaluateFrameCollision(shapeA, frameA, shapeB, frameB, sphere3DToSphere3DEvaluator, sphere3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateShape3DBox3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Box3DReadOnly shapeB,
                                                                         ReferenceFrame frameB)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         return evaluatePointShape3DBox3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB);
      else if (shapeA instanceof Sphere3DReadOnly)
         return evaluateSphere3DBox3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB);
      else
         return evaluateShape3DBox3DCollisionGJK(shapeA, frameA, shapeB, frameB);
   }

   static KinematicsCollisionResult evaluateShape3DBox3DCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, Box3DReadOnly shapeB, ReferenceFrame frameB)
   {
      return evaluateFrameCollisionGJK(shapeA, frameA, shapeB, frameB, box3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateShape3DCapsule3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB,
                                                                             ReferenceFrame frameB)
   {
      if (shapeA instanceof Capsule3DReadOnly)
         return swapShapes(evaluateCapsule3DCapsule3DCollision(shapeB, frameB, (Capsule3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof PointShape3DReadOnly)
         return evaluatePointShape3DCapsule3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB);
      else if (shapeA instanceof Sphere3DReadOnly)
         return evaluateSphere3DCapsule3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB);
      else
         return evaluateShape3DCapsule3DCollisionGJK(shapeA, frameA, shapeB, frameB);
   }

   static KinematicsCollisionResult evaluateShape3DCapsule3DCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, Capsule3DReadOnly shapeB,
                                                                         ReferenceFrame frameB)
   {
      return evaluateFrameCollisionGJK(shapeA, frameA, shapeB, frameB, capsule3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateShape3DCylinder3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Cylinder3DReadOnly shapeB,
                                                                              ReferenceFrame frameB)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         return evaluatePointShape3DCylinder3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB);
      else if (shapeA instanceof Sphere3DReadOnly)
         return evaluateSphere3DCylinder3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB);
      else
         return evaluateShape3DCylinder3DCollisionGJK(shapeA, frameA, shapeB, frameB);
   }

   static KinematicsCollisionResult evaluateShape3DCylinder3DCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, Cylinder3DReadOnly shapeB,
                                                                          ReferenceFrame frameB)
   {
      return evaluateFrameCollisionGJK(shapeA, frameA, shapeB, frameB, cylinder3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateShape3DEllipsoid3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Ellipsoid3DReadOnly shapeB,
                                                                               ReferenceFrame frameB)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         return evaluatePointShape3DEllipsoid3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB);
      else if (shapeA instanceof Sphere3DReadOnly)
         return evaluateSphere3DEllipsoid3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB);
      else
         return evaluateShape3DEllipsoid3DCollisionGJK(shapeA, frameA, shapeB, frameB);
   }

   static KinematicsCollisionResult evaluateShape3DEllipsoid3DCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, Ellipsoid3DReadOnly shapeB,
                                                                           ReferenceFrame frameB)
   {
      return evaluateFrameCollisionGJK(shapeA, frameA, shapeB, frameB, ellipsoid3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateShape3DPointShape3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, PointShape3DReadOnly shapeB,
                                                                                ReferenceFrame frameB)
   {
      if (shapeA instanceof Box3DReadOnly)
         return swapShapes(evaluatePointShape3DBox3DCollision(shapeB, frameB, (Box3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Capsule3DReadOnly)
         return swapShapes(evaluatePointShape3DCapsule3DCollision(shapeB, frameB, (Capsule3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Cylinder3DReadOnly)
         return swapShapes(evaluatePointShape3DCylinder3DCollision(shapeB, frameB, (Cylinder3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Ellipsoid3DReadOnly)
         return swapShapes(evaluatePointShape3DEllipsoid3DCollision(shapeB, frameB, (Ellipsoid3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof PointShape3DReadOnly)
         return swapShapes(evaluatePointShape3DPointShape3DCollision(shapeB, frameB, (PointShape3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Ramp3DReadOnly)
         return swapShapes(evaluatePointShape3DRamp3DCollision(shapeB, frameB, (Ramp3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Sphere3DReadOnly)
         return swapShapes(evaluatePointShape3DSphere3DCollision(shapeB, frameB, (Sphere3DReadOnly) shapeA, frameA));
      else
         return evaluateShape3DPointShape3DCollisionGJK(shapeA, frameA, shapeB, frameB);
   }

   static KinematicsCollisionResult evaluateShape3DPointShape3DCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, PointShape3DReadOnly shapeB,
                                                                            ReferenceFrame frameB)
   {
      return evaluateFrameCollisionGJK(shapeA, frameA, shapeB, frameB, pointShape3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateShape3DRamp3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Ramp3DReadOnly shapeB,
                                                                          ReferenceFrame frameB)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         return evaluatePointShape3DRamp3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB);
      else if (shapeA instanceof Sphere3DReadOnly)
         return evaluateSphere3DRamp3DCollision((Sphere3DReadOnly) shapeA, frameA, shapeB, frameB);
      else
         return evaluateShape3DRamp3DCollisionGJK(shapeA, frameA, shapeB, frameB);
   }

   static KinematicsCollisionResult evaluateShape3DRamp3DCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, Ramp3DReadOnly shapeB,
                                                                      ReferenceFrame frameB)
   {
      return evaluateFrameCollisionGJK(shapeA, frameA, shapeB, frameB, ramp3DFrameChanger);
   }

   public static KinematicsCollisionResult evaluateShape3DSphere3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Sphere3DReadOnly shapeB,
                                                                            ReferenceFrame frameB)
   {
      if (shapeA instanceof Box3DReadOnly)
         return swapShapes(evaluateSphere3DBox3DCollision(shapeB, frameB, (Box3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Capsule3DReadOnly)
         return swapShapes(evaluateSphere3DCapsule3DCollision(shapeB, frameB, (Capsule3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Cylinder3DReadOnly)
         return swapShapes(evaluateSphere3DCylinder3DCollision(shapeB, frameB, (Cylinder3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Ellipsoid3DReadOnly)
         return swapShapes(evaluateSphere3DEllipsoid3DCollision(shapeB, frameB, (Ellipsoid3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof PointShape3DReadOnly)
         return evaluatePointShape3DSphere3DCollision((PointShape3DReadOnly) shapeA, frameA, shapeB, frameB);
      else if (shapeA instanceof Ramp3DReadOnly)
         return swapShapes(evaluateSphere3DRamp3DCollision(shapeB, frameB, (Ramp3DReadOnly) shapeA, frameA));
      else if (shapeA instanceof Sphere3DReadOnly)
         return swapShapes(evaluateSphere3DSphere3DCollision(shapeB, frameB, (Sphere3DReadOnly) shapeA, frameA));
      else
         return evaluateShape3DSphere3DCollisionGJK(shapeA, frameA, shapeB, frameB);
   }

   static KinematicsCollisionResult evaluateShape3DSphere3DCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, Sphere3DReadOnly shapeB,
                                                                        ReferenceFrame frameB)
   {
      return evaluateFrameCollisionGJK(shapeA, frameA, shapeB, frameB, sphere3DFrameChanger);
   }

   static <B extends Shape3DReadOnly> KinematicsCollisionResult evaluateFrameCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, B shapeB,
                                                                                          ReferenceFrame frameB, FrameChanger<B> shapeBFrameChanger)
   {
      KinematicsCollisionResult result = new KinematicsCollisionResult();
      new GilbertJohnsonKeerthiCollisionDetector().evaluateCollision(shapeA, shapeBFrameChanger.changeFrame(shapeB, frameB, frameA), result);
      result.setShapeB(shapeB);
      result.setFrameA(frameA);
      result.setFrameB(frameB);
      result.getPointOnA().setReferenceFrame(frameA);
      result.getPointOnB().setReferenceFrame(frameA);
      result.getNormalOnA().setReferenceFrame(frameA);
      result.getNormalOnB().setReferenceFrame(frameA);
      return result;
   }

   public static KinematicsCollisionResult evaluateShape3DShape3DCollision(Shape3DReadOnly shapeA, ReferenceFrame frameA, Shape3DReadOnly shapeB,
                                                                           ReferenceFrame frameB)
   {
      if (shapeB instanceof Box3DReadOnly)
         return evaluateShape3DBox3DCollision(shapeA, frameA, (Box3DReadOnly) shapeB, frameB);
      else if (shapeB instanceof Capsule3DReadOnly)
         return evaluateShape3DCapsule3DCollision(shapeA, frameA, (Capsule3DReadOnly) shapeB, frameB);
      else if (shapeB instanceof Cylinder3DReadOnly)
         return evaluateShape3DCylinder3DCollision(shapeA, frameA, (Cylinder3DReadOnly) shapeB, frameB);
      else if (shapeB instanceof Ellipsoid3DReadOnly)
         return evaluateShape3DEllipsoid3DCollision(shapeA, frameA, (Ellipsoid3DReadOnly) shapeB, frameB);
      else if (shapeB instanceof PointShape3DReadOnly)
         return evaluateShape3DPointShape3DCollision(shapeA, frameA, (PointShape3DReadOnly) shapeB, frameB);
      else if (shapeB instanceof Ramp3DReadOnly)
         return evaluateShape3DRamp3DCollision(shapeA, frameA, (Ramp3DReadOnly) shapeB, frameB);
      else if (shapeB instanceof Sphere3DReadOnly)
         return evaluateShape3DSphere3DCollision(shapeA, frameA, (Sphere3DReadOnly) shapeB, frameB);
      else
         return evaluateShape3DShape3DCollisionGJK(shapeA, frameA, shapeB, frameB);

   }

   static KinematicsCollisionResult evaluateShape3DShape3DCollisionGJK(Shape3DReadOnly shapeA, ReferenceFrame frameA, Shape3DReadOnly shapeB,
                                                                       ReferenceFrame frameB)
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
      KinematicsCollisionResult result = new KinematicsCollisionResult();
      new GilbertJohnsonKeerthiCollisionDetector().evaluateCollision(shapeA, localShapeB, result);
      result.setShapeA(shapeA);
      result.setShapeB(shapeB);
      result.setFrameA(frameA);
      result.setFrameB(frameB);
      result.getPointOnA().setReferenceFrame(frameA);
      result.getPointOnB().setReferenceFrame(frameA);
      result.getNormalOnA().setReferenceFrame(frameA);
      result.getNormalOnB().setReferenceFrame(frameA);
      return result;
   }

   private static KinematicsCollisionResult swapShapes(KinematicsCollisionResult resultToModify)
   {
      resultToModify.swapShapes();
      return resultToModify;
   }
}
