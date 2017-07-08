package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * <p>
 * Title: FramePoint2d
 * </p>
 *
 * <p>
 * Description: A FramePoint2d is a normal point associated with a specified reference frame
 * </p>
 *
 * @author Learning Locomotion Team
 */
public class FramePoint2D extends FrameTuple2D<FramePoint2D, Point2D> implements FramePoint2DReadOnly, Point2DBasics
{
   private static final long serialVersionUID = -1287148635726098768L;

   /**
    * Creates a new frame point and initializes it coordinates to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FramePoint2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame point and initializes it coordinates to zero and its reference frame to
    * the {@code referenceFrame}.
    * 
    * @param referenceFrame the initial frame for this frame point.
    */
   public FramePoint2D(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Point2D());
   }

   /**
    * Creates a new frame point and initializes it with the given coordinates and the given
    * reference frame.
    * 
    * @param referenceFrame the initial frame for this frame point.
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, double x, double y)
   {
      super(referenceFrame, new Point2D(x, y));
   }

   /**
    * Creates a new frame point and initializes its coordinates {@code x}, {@code y} in order from
    * the given array and initializes its reference frame.
    * 
    * @param referenceFrame the initial frame for this frame point.
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, double[] pointArray)
   {
      super(referenceFrame, new Point2D(pointArray));
   }

   /**
    * Creates a new frame point and initializes it to {@code tuple2DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param tuple2DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      super(referenceFrame, new Point2D(tuple2DReadOnly));
   }

   /**
    * Creates a new frame point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FramePoint2D(FrameTuple2DReadOnly other)
   {
      super(other.getReferenceFrame(), new Point2D(other));
   }

   /**
    * Creates a new frame point and initializes it to the x and y coordinates of
    * {@code frameTuple3DReadOnly}.
    *
    * @param frameTuple3DReadOnly the tuple to copy the coordinates and reference frame from. Not
    *           modified.
    */
   public FramePoint2D(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      this(frameTuple3DReadOnly.getReferenceFrame(), new Point2D(frameTuple3DReadOnly));
   }

   public static FramePoint2D generateRandomFramePoint2d(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax)
   {
      FramePoint2D randomPoint = new FramePoint2D(zUpFrame, RandomNumbers.nextDouble(random, xMin, xMax), RandomNumbers.nextDouble(random, yMin, yMax));

      return randomPoint;
   }

   /**
    * Gets the read-only reference to the point used in {@code this}.
    *
    * @return the point of {@code this}.
    */
   public Point2D getPoint()
   {
      return tuple;
   }
}
