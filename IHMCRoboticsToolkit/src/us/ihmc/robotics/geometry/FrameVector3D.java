package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.random.RandomGeometry;

/**
 * {@code FrameVector3D} is a 3D vector expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Vector3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector3D}. This allows, for instance, to enforce, at runtime, that operations on
 * vectors occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of vector in different
 * reference frame.
 * </p>
 * <p>
 * Because a {@code FrameVector3D} extends {@code FrameVector3DReadOnly}, it is compatible with
 * methods only requiring {@code FrameVector3DReadOnly}. However, these methods do NOT assert that
 * the operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameVector3D}.
 * </p>
 */
public class FrameVector3D extends FrameTuple3D<FrameVector3D, Vector3D> implements FrameVector3DReadOnly, Vector3DBasics
{
   private static final long serialVersionUID = -4475317718392284548L;

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameVector3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame vector and initializes it components to zero and its reference frame to
    * the {@code referenceFrame}.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    */
   public FrameVector3D(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Vector3D());
   }

   /**
    * Creates a new frame vector and initializes it with the given components and the given
    * reference frame.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    */
   public FrameVector3D(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(referenceFrame, new Vector3D(x, y, z));
   }

   /**
    * Creates a new frame vector and initializes its component {@code x}, {@code y}, {@code z} in
    * order from the given array and initializes its reference frame.
    * 
    * @param referenceFrame the initial frame for this frame vector.
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public FrameVector3D(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      super(referenceFrame, new Vector3D(vectorArray));
   }

   /**
    * Creates a new frame vector and initializes it to {@code tuple3DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame vector.
    * @param tuple3DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameVector3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      super(referenceFrame, new Vector3D(tuple3DReadOnly));
   }

   /**
    * Creates a new frame vector and initializes its reference frame x and y components from
    * {@code frameTuple2DReadOnly}.
    *
    * @param frameTuple2DReadOnly the tuple to copy the components and reference frame from. Not
    *           modified.
    */
   public FrameVector3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(frameTuple2DReadOnly.getReferenceFrame(), new Vector3D(frameTuple2DReadOnly));
   }

   /**
    * Creates a new frame vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameVector3D(FrameTuple3DReadOnly other)
   {
      super(other.getReferenceFrame(), new Vector3D(other));
   }

   public static FrameVector3D generateRandomFrameVector(Random random, ReferenceFrame frame)
   {
      FrameVector3D randomVector = new FrameVector3D(frame, RandomGeometry.nextVector3D(random));
      return randomVector;
   }

   public static FrameVector3D generateRandomFrameVector(Random random, ReferenceFrame frame, double xMin, double xMax, double yMin, double yMax, double zMin,
                                                         double zMax)
   {
      FrameVector3D randomVector = new FrameVector3D(frame, RandomNumbers.nextDouble(random, xMin, xMax), RandomNumbers.nextDouble(random, yMin, yMax),
                                                     RandomNumbers.nextDouble(random, zMin, zMax));
      return randomVector;
   }

   /**
    * Sets this frame vector to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame vector to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public void setAndNormalize(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNormalize(other);
   }

   /**
    * Sets this frame vector to the cross product of {@code this} and {@code other}.
    * <p>
    * this = this &times; other
    * </p>
    *
    * @param other the second frame vector in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public void cross(FrameVector3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.cross(tuple, other);
   }

   /**
    * Sets this frame vector to the cross product of {@code this} and {@code frameTuple3DReadOnly}.
    * <p>
    * this = this &times; frameTuple3DReadOnly
    * </p>
    *
    * @param frameTuple3DReadOnly the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple3DReadOnly} is not expressed in
    *            the same reference frame as {@code this}.
    */
   public void cross(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple3DReadOnly);
      tuple.cross(tuple, frameTuple3DReadOnly);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameVector1} and {@code frameVector2}.
    * <p>
    * this = frameVector1 &times; frameVector2
    * </p>
    *
    * @param frameVector1 the first frame vector in the cross product. Not modified.
    * @param frameVector2 the second frame vector in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameVector1} or {@code frameVector2}
    *            is not expressed in the same reference frame as {@code this}.
    */
   public void cross(FrameVector3DReadOnly frameVector1, FrameVector3DReadOnly frameVector2)
   {
      cross((FrameTuple3DReadOnly) frameVector1, (FrameTuple3DReadOnly) frameVector2);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameTuple1} and {@code frameTuple2}.
    * <p>
    * this = frameTuple1 &times; frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple in the cross product. Not modified.
    * @param frameTuple2 the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same reference frame as {@code this}.
    */
   public void cross(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      tuple.cross(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameTuple1} and {@code tuple2}.
    * <p>
    * this = frameTuple1 &times; tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple in the cross product. Not modified.
    * @param tuple2 the second tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public void cross(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      tuple.cross(frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the cross product of {@code tuple1} and {@code frameTuple2}.
    * <p>
    * this = tuple1 &times; frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple in the cross product. Not modified.
    * @param frameTuple2 the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public void cross(Tuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      tuple.cross(frameTuple1, frameTuple2);
   }

   /**
    * Gets the read-only reference to the vector used in {@code this}.
    *
    * @return the vector of {@code this}.
    */
   public Vector3D getVector()
   {
      return tuple;
   }
}
