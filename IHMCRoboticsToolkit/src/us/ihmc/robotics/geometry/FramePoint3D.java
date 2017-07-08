package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FramePoint3D} is a 3D point expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Point3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePoint3D}. This allows, for instance, to enforce, at runtime, that operations on
 * vectors occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a point in different
 * reference frame.
 * </p>
 * <p>
 * Because a {@code FramePoint3D} extends {@code Point3DBasics}, it is compatible with methods only
 * requiring {@code Point3DBasics}. However, these methods do NOT assert that the operation occur in
 * the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameVector3D}.
 * </p>
 */
public class FramePoint3D extends FrameTuple3D<FramePoint3D, Point3D> implements FramePoint3DReadOnly, Point3DBasics
{
   private static final long serialVersionUID = -4831948077397801540L;

   /**
    * Creates a new frame point and initializes it coordinates to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FramePoint3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame point and initializes it coordinates to zero and its reference frame to
    * the {@code referenceFrame}.
    * 
    * @param referenceFrame the initial frame for this frame point.
    */
   public FramePoint3D(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Point3D());
   }

   /**
    * Creates a new frame point and initializes it with the given coordinates and the given
    * reference frame.
    * 
    * @param referenceFrame the initial frame for this frame point.
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    * @param z the z-coordinate.
    */
   public FramePoint3D(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(referenceFrame, new Point3D(x, y, z));
   }

   /**
    * Creates a new frame point and initializes its coordinates {@code x}, {@code y}, {@code z} in
    * order from the given array and initializes its reference frame.
    * 
    * @param referenceFrame the initial frame for this frame point.
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public FramePoint3D(ReferenceFrame referenceFrame, double[] pointArray)
   {
      super(referenceFrame, new Point3D(pointArray));
   }

   /**
    * Creates a new frame point and initializes it to {@code tuple3DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param tuple3DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FramePoint3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      super(referenceFrame, new Point3D(tuple3DReadOnly));
   }

   /**
    * Creates a new frame point and initializes its reference frame x and y coordinates from
    * {@code frameTuple2DReadOnly}.
    *
    * @param frameTuple2DReadOnly the tuple to copy the coordinates and reference frame from. Not
    *           modified.
    */
   public FramePoint3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(frameTuple2DReadOnly.getReferenceFrame(), new Point3D(frameTuple2DReadOnly));
   }

   /**
    * Creates a new frame point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the coordinates and reference frame from. Not modified.
    */
   public FramePoint3D(FrameTuple3DReadOnly other)
   {
      super(other.getReferenceFrame(), new Point3D(other));
   }

   public static FramePoint3D generateRandomFramePoint(Random random, ReferenceFrame frame, double xMaxAbsoluteX, double yMaxAbsoluteY, double zMaxAbsoluteZ)
   {
      FramePoint3D randomPoint = new FramePoint3D(frame, RandomNumbers.nextDouble(random, xMaxAbsoluteX), RandomNumbers.nextDouble(random, yMaxAbsoluteY),
                                                  RandomNumbers.nextDouble(random, zMaxAbsoluteZ));
      return randomPoint;
   }

   public static FramePoint3D generateRandomFramePoint(Random random, ReferenceFrame frame, double xMin, double xMax, double yMin, double yMax, double zMin,
                                                       double zMax)
   {
      FramePoint3D randomPoint = new FramePoint3D(frame, RandomNumbers.nextDouble(random, xMin, xMax), RandomNumbers.nextDouble(random, yMin, yMax),
                                                  RandomNumbers.nextDouble(random, zMin, zMax));
      return randomPoint;
   }

   /**
    * Gets the read-only reference to the point used in {@code this}.
    *
    * @return the point of {@code this}.
    */
   public Point3D getPoint()
   {
      return tuple;
   }
}
