package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
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
public class FramePoint2d extends FrameTuple2D<FramePoint2d, Point2D> implements FramePoint2DReadOnly, Point2DBasics
{
   private static final long serialVersionUID = -1287148635726098768L;

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Point2D());
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2d(ReferenceFrame referenceFrame, double x, double y)
   {
      super(referenceFrame, new Point2D(x, y));
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2d(ReferenceFrame referenceFrame, double[] pointArray)
   {
      super(referenceFrame, new Point2D(pointArray));
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2d(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      super(referenceFrame, new Point2D(tuple2DReadOnly));
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2d(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(frameTuple2DReadOnly.getReferenceFrame(), new Point2D(frameTuple2DReadOnly));
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2d(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      this(frameTuple3DReadOnly.getReferenceFrame(), new Point2D(frameTuple3DReadOnly));
   }

   public static FramePoint2d generateRandomFramePoint2d(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax)
   {
      FramePoint2d randomPoint = new FramePoint2d(zUpFrame, RandomNumbers.nextDouble(random, xMin, xMax), RandomNumbers.nextDouble(random, yMin, yMax));

      return randomPoint;
   }

   /**
    * Returns the point in this FramePoint2d.
    *
    * @return Point2d
    */
   public Point2DReadOnly getPoint()
   {
      return tuple;
   }
}
