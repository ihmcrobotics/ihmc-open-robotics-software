package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;

/**
 * Read-only interface adding {@code ReferenceFrame} support to
 * {@link EuclidShape3DCollisionResultReadOnly}.
 *
 * @author Sylvain Bertrand
 */
public interface EuclidFrameShape3DCollisionResultReadOnly extends EuclidShape3DCollisionResultReadOnly
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getPointOnA();

   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getPointOnB();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getNormalOnA();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getNormalOnB();

   /**
    * Gets, if available, the reference frame in which the shape A is expressed.
    *
    * @return the frame of shape A.
    */
   ReferenceFrame getFrameA();

   /**
    * Gets, if available, the reference frame in which the shape B is expressed.
    *
    * @return the frame of shape B.
    */
   ReferenceFrame getFrameB();

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other collision result to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two collision results are equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean epsilonEquals(EuclidFrameShape3DCollisionResultReadOnly other, double epsilon)
   {
      if (getFrameA() == null ? other.getFrameA() != null : getFrameA() != other.getFrameA())
         return false;
      if (getFrameB() == null ? other.getFrameB() != null : getFrameB() != other.getFrameB())
         return false;
      return EuclidShape3DCollisionResultReadOnly.super.epsilonEquals(other, epsilon);
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
   default boolean geometricallyEquals(EuclidFrameShape3DCollisionResultReadOnly other, double epsilon)
   {
      if (areShapesColliding() != other.areShapesColliding())
         return false;

      if (!EuclidCoreTools.epsilonEquals(getSignedDistance(), other.getSignedDistance(), epsilon))
         return false;

      boolean swap;
      if (getShapeA() != null || getShapeB() != null || other.getShapeA() != null || other.getShapeB() != null)
         swap = getShapeA() != other.getShapeA();
      else
         swap = !getPointOnA().geometricallyEquals(other.getPointOnA(), epsilon);

      Shape3DReadOnly otherShapeA = swap ? other.getShapeB() : other.getShapeA();
      Shape3DReadOnly otherShapeB = swap ? other.getShapeA() : other.getShapeB();
      FramePoint3DReadOnly otherPointOnA = swap ? other.getPointOnB() : other.getPointOnA();
      FramePoint3DReadOnly otherPointOnB = swap ? other.getPointOnA() : other.getPointOnB();
      FrameVector3DReadOnly otherNormalOnA = swap ? other.getNormalOnB() : other.getNormalOnA();
      FrameVector3DReadOnly otherNormalOnB = swap ? other.getNormalOnA() : other.getNormalOnB();
      ReferenceFrame otherFrameA = swap ? other.getFrameB() : other.getFrameA();
      ReferenceFrame otherFrameB = swap ? other.getFrameA() : other.getFrameB();

      if (getShapeA() != otherShapeA)
         return false;
      if (getShapeB() != otherShapeB)
         return false;

      if (getPointOnA().containsNaN() ? !otherPointOnA.containsNaN() : !getPointOnA().geometricallyEquals(otherPointOnA, epsilon))
         return false;
      if (getPointOnB().containsNaN() ? !otherPointOnB.containsNaN() : !getPointOnB().geometricallyEquals(otherPointOnB, epsilon))
         return false;

      if (getNormalOnA().containsNaN() ? !otherNormalOnA.containsNaN() : !getNormalOnA().geometricallyEquals(otherNormalOnA, epsilon))
         return false;
      if (getNormalOnB().containsNaN() ? !otherNormalOnB.containsNaN() : !getNormalOnB().geometricallyEquals(otherNormalOnB, epsilon))
         return false;

      if (getFrameA() != null)
         getFrameA().checkReferenceFrameMatch(otherFrameA);
      else if (otherFrameA != null)
         return false;
      if (getFrameB() != null)
         getFrameB().checkReferenceFrameMatch(otherFrameA);
      else if (otherFrameB != null)
         return false;

      return true;
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
   default boolean equals(EuclidFrameShape3DCollisionResultReadOnly other)
   {
      if (other == this)
         return true;
      if (other == null)
         return false;

      if (getFrameA() == null ? other.getFrameA() != null : getFrameA() != other.getFrameA())
         return false;
      if (getFrameB() == null ? other.getFrameB() != null : getFrameB() != other.getFrameB())
         return false;

      return EuclidShape3DCollisionResultReadOnly.super.equals(other);
   }
}
