package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * One of the main goals of this class is to check, at runtime, that operations on vectors occur
 * within the same Frame. This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class FrameVector2d extends FrameTuple2D<FrameVector2d, Vector2D> implements FrameVector2DReadOnly, Vector2DBasics
{
   private static final long serialVersionUID = -610124454205790361L;

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Vector2D());
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(ReferenceFrame referenceFrame, double x, double y)
   {
      super(referenceFrame, new Vector2D(x, y));
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      super(referenceFrame, new Vector2D(tuple2DReadOnly));
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      this(referenceFrame, new Vector2D(vectorArray));
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      this(frameTuple2DReadOnly.getReferenceFrame(), new Vector2D(frameTuple2DReadOnly));
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FrameVector2d(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      this(frameTuple3DReadOnly.getReferenceFrame(), new Vector2D(frameTuple3DReadOnly));
   }

   public static FrameVector2d generateRandomFrameVector2d(Random random, ReferenceFrame zUpFrame)
   {
      double randomAngle = RandomNumbers.nextDouble(random, -Math.PI, Math.PI);

      FrameVector2d randomVector = new FrameVector2d(zUpFrame, Math.cos(randomAngle), Math.sin(randomAngle));

      return randomVector;
   }

   public void setAndNormalize(FrameVector2DReadOnly other)
   {
      tuple.setAndNormalize(other);
   }

   public double cross(FrameVector2d frameVector2D)
   {
      return FrameVector2DReadOnly.super.cross(frameVector2D);
   }

   /**
    * Returns the vector inside this FrameVector.
    *
    * @return Vector2d
    */
   public Vector2DReadOnly getVector()
   {
      return this.tuple;
   }
}
