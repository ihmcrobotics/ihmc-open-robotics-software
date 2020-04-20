package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;

/**
 * Write and read interface adding {@code ReferenceFrame} support to
 * {@link EuclidShape3DCollisionResultBasics}.
 *
 * @author Sylvain Bertrand
 */
public interface EuclidFrameShape3DCollisionResultBasics extends EuclidFrameShape3DCollisionResultReadOnly, EuclidShape3DCollisionResultBasics
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DBasics getPointOnA();

   /** {@inheritDoc} */
   @Override
   FramePoint3DBasics getPointOnB();

   /** {@inheritDoc} */
   @Override
   FrameVector3DBasics getNormalOnA();

   /** {@inheritDoc} */
   @Override
   FrameVector3DBasics getNormalOnB();

   void setFrameA(ReferenceFrame frameA);

   void setFrameB(ReferenceFrame frameB);

   /**
    * Copies the values from {@code other} into {@code this}.
    * 
    * @param other the other object to copy the values from. Not modified.
    */
   default void set(EuclidFrameShape3DCollisionResultReadOnly other)
   {
      setShapeA(other.getShapeA());
      setShapeB(other.getShapeB());
      setShapesAreColliding(other.areShapesColliding());
      setSignedDistance(other.getSignedDistance());
      getPointOnA().setIncludingFrame(other.getPointOnA());
      getNormalOnA().setIncludingFrame(other.getNormalOnA());
      getPointOnB().setIncludingFrame(other.getPointOnB());
      getNormalOnB().setIncludingFrame(other.getNormalOnB());
      setFrameA(other.getFrameA());
      setFrameB(other.getFrameB());
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      setShapesAreColliding(false);
      setSignedDistance(0.0);
      getPointOnA().setToZero();
      getNormalOnA().setToZero();
      getPointOnB().setToZero();
      getNormalOnB().setToZero();
      setFrameA(null);
      setFrameB(null);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      setShapesAreColliding(false);
      setSignedDistance(Double.NaN);
      getPointOnA().setToNaN();
      getNormalOnA().setToNaN();
      getPointOnB().setToNaN();
      getNormalOnB().setToNaN();
      setFrameA(null);
      setFrameB(null);
   }

   @Override
   default void swapShapes()
   {
      Shape3DReadOnly tempShape = getShapeA();
      setShapeA(getShapeB());
      setShapeB(tempShape);

      double tempX = getPointOnA().getX();
      double tempY = getPointOnA().getY();
      double tempZ = getPointOnA().getZ();
      ReferenceFrame tempFrame = getPointOnA().getReferenceFrame();

      getPointOnA().setIncludingFrame(getPointOnB());
      getPointOnB().setIncludingFrame(tempFrame, tempX, tempY, tempZ);

      tempX = getNormalOnA().getX();
      tempY = getNormalOnA().getY();
      tempZ = getNormalOnA().getZ();
      tempFrame = getNormalOnA().getReferenceFrame();
      getNormalOnA().setIncludingFrame(getNormalOnB());
      getNormalOnB().set(tempFrame, tempX, tempY, tempZ);

      tempFrame = getFrameA();
      setFrameA(getFrameB());
      setFrameB(tempFrame);
   }

}
