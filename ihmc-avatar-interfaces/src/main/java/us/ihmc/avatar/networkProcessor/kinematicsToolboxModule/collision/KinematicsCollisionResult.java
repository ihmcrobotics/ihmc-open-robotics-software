package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;

/**
 * Class for holding the result of a collision query between two shapes and their respective
 * reference frame information.
 * 
 * @author Sylvain Bertrand
 */
public class KinematicsCollisionResult implements EuclidShape3DCollisionResultBasics, EpsilonComparable<KinematicsCollisionResult>,
      GeometricallyComparable<KinematicsCollisionResult>
{
   /** Whether the shapes are colliding. */
   private boolean shapesAreColliding;
   /** The collision distance, either separation distance or penetration depth. */
   private double signedDistance;

   private KinematicsCollidable collidableA;
   private KinematicsCollidable collidableB;

   /** The first shape in the collision. */
   private Shape3DReadOnly shapeA;
   /** The second shape in the collision. */
   private Shape3DReadOnly shapeB;

   /** The reference frame in which {@code shapeA} is expressed. */
   private ReferenceFrame frameA;
   /** The reference frame in which {@code shapeB} is expressed. */
   private ReferenceFrame frameB;

   /** The key point on the shape A. */
   private final FramePoint3D pointOnA = new FramePoint3D();
   /** The surface normal at {@code pointOnA}. */
   private final FrameVector3D normalOnA = new FrameVector3D();

   /** The key point on the shape B. */
   private final FramePoint3D pointOnB = new FramePoint3D();
   /** The surface normal at {@code pointOnB}. */
   private final FrameVector3D normalOnB = new FrameVector3D();

   /**
    * Creates a new empty collision result.
    */
   public KinematicsCollisionResult()
   {
   }

   /** {@inheritDoc} */
   @Override
   public void setShapesAreColliding(boolean shapesAreColliding)
   {
      this.shapesAreColliding = shapesAreColliding;
   }

   /** {@inheritDoc} */
   @Override
   public void setSignedDistance(double distance)
   {
      this.signedDistance = distance;
   }

   public void setCollidableA(KinematicsCollidable collidableA)
   {
      this.collidableA = collidableA;
   }

   public void setCollidableB(KinematicsCollidable collidableB)
   {
      this.collidableB = collidableB;
   }

   /** {@inheritDoc} */
   @Override
   public void setShapeA(Shape3DReadOnly shapeA)
   {
      this.shapeA = shapeA;
   }

   /** {@inheritDoc} */
   @Override
   public void setShapeB(Shape3DReadOnly shapeB)
   {
      this.shapeB = shapeB;
   }

   /**
    * Sets the reference frame in which {@code shapeA} is expressed.
    * <p>
    * This does <b>not</b> change the reference frame of {@code pointOnA} nor {@code normalOnA}.
    * </p>
    * 
    * @param frameA the reference frame for {@code shapeA}.
    */
   public void setFrameA(ReferenceFrame frameA)
   {
      this.frameA = frameA;
   }

   /**
    * Sets the reference frame in which {@code shapeB} is expressed.
    * <p>
    * This does <b>not</b> change the reference frame of {@code pointOnB} nor {@code normalOnB}.
    * </p>
    * 
    * @param frameB the reference frame for {@code shapeB}.
    */
   public void setFrameB(ReferenceFrame frameB)
   {
      this.frameB = frameB;
   }

   /** {@inheritDoc} */
   @Override
   public boolean areShapesColliding()
   {
      return shapesAreColliding;
   }

   /** {@inheritDoc} */
   @Override
   public double getSignedDistance()
   {
      return signedDistance;
   }

   public KinematicsCollidable getCollidableA()
   {
      return collidableA;
   }

   public KinematicsCollidable getCollidableB()
   {
      return collidableB;
   }

   /** {@inheritDoc} */
   @Override
   public Shape3DReadOnly getShapeA()
   {
      return shapeA;
   }

   /** {@inheritDoc} */
   @Override
   public Shape3DReadOnly getShapeB()
   {
      return shapeB;
   }

   /**
    * Gets the reference frame in which {@code shapeA} is expressed.
    * 
    * @return the reference frame for {@code shapeA}.
    */
   public ReferenceFrame getFrameA()
   {
      return frameA;
   }
   
   /**
    * Gets the reference frame in which {@code shapeB} is expressed.
    * 
    * @return the reference frame for {@code shapeB}.
    */
   public ReferenceFrame getFrameB()
   {
      return frameB;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3D getPointOnA()
   {
      return pointOnA;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3D getNormalOnA()
   {
      return normalOnA;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3D getPointOnB()
   {
      return pointOnB;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3D getNormalOnB()
   {
      return normalOnB;
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    * <p>
    * Two instances of collision frame results are not considered equal when their respective frames
    * are different.
    * </p>
    * 
    * @param other   the other collision result to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two collision results are equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(KinematicsCollisionResult other, double epsilon)
   {
      if (frameA == null ? other.frameA != null : frameA != other.frameA)
         return false;
      if (frameB == null ? other.frameB != null : frameB != other.frameB)
         return false;
      return EuclidShape3DCollisionResultBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests each feature of {@code this} against {@code other} for geometric similarity.
    * 
    * @param other   the other collision result to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each feature.
    * @return {@code true} if the two collision results are considered geometrically similar,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} does not hold the same reference frames
    *                                         as {@code this}.
    */
   @Override
   public boolean geometricallyEquals(KinematicsCollisionResult other, double epsilon)
   {
      if (frameA != null)
         frameA.checkReferenceFrameMatch(other.frameA);
      else if (other.frameA != null)
         return false;
      if (frameB != null)
         frameB.checkReferenceFrameMatch(other.frameB);
      else if (other.frameB != null)
         return false;
      return EuclidShape3DCollisionResultBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidShape3DCollisionResultReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof KinematicsCollisionResult)
         return equals((KinematicsCollisionResult) object);
      if (object instanceof EuclidShape3DCollisionResultReadOnly)
         return EuclidShape3DCollisionResultBasics.super.equals((EuclidShape3DCollisionResultReadOnly) object);
      else
         return false;
   }

   /**
    * Tests on a per component basis, if this collision result is exactly equal to {@code other}.
    * <p>
    * Two instances of collision frame results are not considered equal when their respective frames
    * are different.
    * </p>
    *
    * @param other the other collision result to compare against this. Not modified.
    * @return {@code true} if the two collision results are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(KinematicsCollisionResult other)
   {
      if (other == this)
         return true;
      if (other == null)
         return false;

      if (frameA == null ? other.frameA != null : frameA != other.frameA)
         return false;
      if (frameB == null ? other.frameB != null : frameB != other.frameB)
         return false;

      return EuclidShape3DCollisionResultBasics.super.equals(other);
   }

   /**
    * Provides a {@code String} representation of this collision result as follows:<br>
    * When shapes are colliding:
    * 
    * <pre>
    * Collision test result: colliding, depth: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * 
    * When shapes are not colliding:
    * 
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * 
    * @return the {@code String} representing this collision result.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getEuclidShape3DCollisionResultString(this);
   }
}
