package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.FrameEllipsoid3D;
import us.ihmc.euclid.referenceFrame.FramePointShape3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShapeCollisionTools;
import us.ihmc.euclid.referenceFrame.collision.epa.FrameExpandingPolytopeAlgorithm;
import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class PhysicsEngineTools
{
   public static FrameShape3DBasics toFrameShape3DBasics(ReferenceFrame referenceFrame, Shape3DReadOnly shape)
   {
      if (shape instanceof Box3DReadOnly)
         return new FrameBox3D(referenceFrame, (Box3DReadOnly) shape);
      if (shape instanceof Capsule3DReadOnly)
         return new FrameCapsule3D(referenceFrame, (Capsule3DReadOnly) shape);
      if (shape instanceof ConvexPolytope3DReadOnly)
         return new FrameConvexPolytope3D(referenceFrame, (ConvexPolytope3DReadOnly) shape);
      if (shape instanceof Cylinder3DReadOnly)
         return new FrameCylinder3D(referenceFrame, (Cylinder3DReadOnly) shape);
      if (shape instanceof Ellipsoid3DReadOnly)
         return new FrameEllipsoid3D(referenceFrame, (Ellipsoid3DReadOnly) shape);
      if (shape instanceof PointShape3DReadOnly)
         return new FramePointShape3D(referenceFrame, (PointShape3DReadOnly) shape);
      if (shape instanceof Ramp3DReadOnly)
         return new FrameRamp3D(referenceFrame, (Ramp3DReadOnly) shape);
      if (shape instanceof Sphere3DReadOnly)
         return new FrameSphere3D(referenceFrame, (Sphere3DReadOnly) shape);

      throw new UnsupportedOperationException("Unsupported shape for conversion: " + shape.getClass().getSimpleName());
   }

   public static void evaluateShape3DBox3DCollision(FrameShape3DReadOnly shapeA, FrameBox3DReadOnly shapeB,
                                                    EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      if (shapeA instanceof FramePointShape3DReadOnly)
         EuclidFrameShapeCollisionTools.evaluatePointShape3DBox3DCollision((FramePointShape3DReadOnly) shapeA, shapeB, resultToPack);
      else if (shapeA instanceof FrameSphere3DReadOnly)
         EuclidFrameShapeCollisionTools.evaluateSphere3DBox3DCollision((FrameSphere3DReadOnly) shapeA, shapeB, resultToPack);
      else
         evaluateShape3DShape3DCollisionEPA(shapeA, shapeB, resultToPack);
   }

   public static void evaluateShape3DCapsule3DCollision(FrameShape3DReadOnly shapeA, FrameCapsule3DReadOnly shapeB,
                                                        EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      if (shapeA instanceof FrameCapsule3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluateCapsule3DCapsule3DCollision(shapeB, (FrameCapsule3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FramePointShape3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DCapsule3DCollision((FramePointShape3DReadOnly) shapeA, shapeB, resultToPack);
      }
      else if (shapeA instanceof FrameSphere3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluateSphere3DCapsule3DCollision((FrameSphere3DReadOnly) shapeA, shapeB, resultToPack);
      }
      else
      {
         evaluateShape3DShape3DCollisionEPA(shapeA, shapeB, resultToPack);
      }
   }

   public static void evaluateShape3DCylinder3DCollision(FrameShape3DReadOnly shapeA, FrameCylinder3DReadOnly shapeB,
                                                         EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      if (shapeA instanceof FramePointShape3DReadOnly)
         EuclidFrameShapeCollisionTools.evaluatePointShape3DCylinder3DCollision((FramePointShape3DReadOnly) shapeA, shapeB, resultToPack);
      else if (shapeA instanceof FrameSphere3DReadOnly)
         EuclidFrameShapeCollisionTools.evaluateSphere3DCylinder3DCollision((FrameSphere3DReadOnly) shapeA, shapeB, resultToPack);
      else
         evaluateShape3DShape3DCollisionEPA(shapeA, shapeB, resultToPack);
   }

   public static void evaluateShape3DEllipsoid3DCollision(FrameShape3DReadOnly shapeA, FrameEllipsoid3DReadOnly shapeB,
                                                          EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      if (shapeA instanceof FramePointShape3DReadOnly)
         EuclidFrameShapeCollisionTools.evaluatePointShape3DEllipsoid3DCollision((FramePointShape3DReadOnly) shapeA, shapeB, resultToPack);
      else if (shapeA instanceof Sphere3DReadOnly)
         EuclidFrameShapeCollisionTools.evaluateSphere3DEllipsoid3DCollision((FrameSphere3DReadOnly) shapeA, shapeB, resultToPack);
      else
         evaluateShape3DShape3DCollisionEPA(shapeA, shapeB, resultToPack);
   }

   public static void evaluateShape3DPointShape3DCollision(FrameShape3DReadOnly shapeA, FramePointShape3DReadOnly shapeB,
                                                           EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      if (shapeA instanceof FrameBox3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DBox3DCollision(shapeB, (FrameBox3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameCapsule3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DCapsule3DCollision(shapeB, (FrameCapsule3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameCylinder3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DCylinder3DCollision(shapeB, (FrameCylinder3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameEllipsoid3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DEllipsoid3DCollision(shapeB, (FrameEllipsoid3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FramePointShape3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DPointShape3DCollision(shapeB, (FramePointShape3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameRamp3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DRamp3DCollision(shapeB, (FrameRamp3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameSphere3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DSphere3DCollision(shapeB, (FrameSphere3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else
      {
         evaluateShape3DShape3DCollisionEPA(shapeA, shapeB, resultToPack);
      }
   }

   public static void evaluateShape3DRamp3DCollision(FrameShape3DReadOnly shapeA, FrameRamp3DReadOnly shapeB,
                                                     EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      if (shapeA instanceof PointShape3DReadOnly)
         EuclidFrameShapeCollisionTools.evaluatePointShape3DRamp3DCollision((FramePointShape3DReadOnly) shapeA, shapeB, resultToPack);
      else if (shapeA instanceof Sphere3DReadOnly)
         EuclidFrameShapeCollisionTools.evaluateSphere3DRamp3DCollision((FrameSphere3DReadOnly) shapeA, shapeB, resultToPack);
      else
         evaluateShape3DShape3DCollisionEPA(shapeA, shapeB, resultToPack);
   }

   public static void evaluateShape3DSphere3DCollision(FrameShape3DReadOnly shapeA, FrameSphere3DReadOnly shapeB,
                                                       EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      if (shapeA instanceof FrameBox3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluateSphere3DBox3DCollision(shapeB, (FrameBox3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameCapsule3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluateSphere3DCapsule3DCollision(shapeB, (FrameCapsule3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameCylinder3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluateSphere3DCylinder3DCollision(shapeB, (FrameCylinder3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameEllipsoid3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluateSphere3DEllipsoid3DCollision(shapeB, (FrameEllipsoid3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FramePointShape3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluatePointShape3DSphere3DCollision((FramePointShape3DReadOnly) shapeA, shapeB, resultToPack);
      }
      else if (shapeA instanceof FrameRamp3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluateSphere3DRamp3DCollision(shapeB, (FrameRamp3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else if (shapeA instanceof FrameSphere3DReadOnly)
      {
         EuclidFrameShapeCollisionTools.evaluateSphere3DSphere3DCollision(shapeB, (FrameSphere3DReadOnly) shapeA, resultToPack);
         resultToPack.swapShapes();
      }
      else
      {
         evaluateShape3DShape3DCollisionEPA(shapeA, shapeB, resultToPack);
      }
   }

   public static void evaluateShape3DShape3DCollision(FrameShape3DReadOnly shapeA, FrameShape3DReadOnly shapeB,
                                                      EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      if (shapeB instanceof FrameBox3DReadOnly)
         evaluateShape3DBox3DCollision(shapeA, (FrameBox3DReadOnly) shapeB, resultToPack);
      else if (shapeB instanceof FrameCapsule3DReadOnly)
         evaluateShape3DCapsule3DCollision(shapeA, (FrameCapsule3DReadOnly) shapeB, resultToPack);
      else if (shapeB instanceof FrameCylinder3DReadOnly)
         evaluateShape3DCylinder3DCollision(shapeA, (FrameCylinder3DReadOnly) shapeB, resultToPack);
      else if (shapeB instanceof FrameEllipsoid3DReadOnly)
         evaluateShape3DEllipsoid3DCollision(shapeA, (FrameEllipsoid3DReadOnly) shapeB, resultToPack);
      else if (shapeB instanceof FramePointShape3DReadOnly)
         evaluateShape3DPointShape3DCollision(shapeA, (FramePointShape3DReadOnly) shapeB, resultToPack);
      else if (shapeB instanceof FrameRamp3DReadOnly)
         evaluateShape3DRamp3DCollision(shapeA, (FrameRamp3DReadOnly) shapeB, resultToPack);
      else if (shapeB instanceof FrameSphere3DReadOnly)
         evaluateShape3DSphere3DCollision(shapeA, (FrameSphere3DReadOnly) shapeB, resultToPack);
      else
         evaluateShape3DShape3DCollisionEPA(shapeA, shapeB, resultToPack);

   }

   static void evaluateShape3DShape3DCollisionEPA(FrameShape3DReadOnly shapeA, FrameShape3DReadOnly shapeB,
                                                  EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      new FrameExpandingPolytopeAlgorithm().evaluateCollision(shapeA, shapeB, resultToPack);
   }

   public static int computeCollisionHashCode(Collidable collidableA, Collidable collidableB)
   {
      int collidableAID = collidableA.hashCode();
      int collidableBID = collidableB.hashCode();
      if (collidableAID > collidableBID)
         return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.combineHashCode(collidableAID, collidableBID));
      else
         return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.combineHashCode(collidableBID, collidableAID));
   }

   public static String collidableSimpleName(Collidable collidable)
   {
      return collidable.getRigidBody() != null ? collidable.getRigidBody().getName() : "static";
   }
}
