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
 * One of the main goals of this class is to check, at runtime, that operations on points occur
 * within the same Frame. This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class FramePoint extends FrameTuple3D<FramePoint, Point3D> implements FramePoint3DReadOnly, Point3DBasics
{
   private static final long serialVersionUID = -4831948077397801540L;

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      super(referenceFrame, new Point3D(tuple3DReadOnly));
   }

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint(ReferenceFrame referenceFrame, double[] pointArray)
   {
      super(referenceFrame, new Point3D(pointArray));
   }

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Point3D());
   }

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint(FrameTuple3DReadOnly frameTuple)
   {
      super(frameTuple.getReferenceFrame(), new Point3D(frameTuple));
   }

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint(FrameTuple2DReadOnly frameTuple2d)
   {
      super(frameTuple2d.getReferenceFrame(), new Point3D());
      set(frameTuple2d);
   }

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(referenceFrame, new Point3D(x, y, z));
   }

   public static FramePoint generateRandomFramePoint(Random random, ReferenceFrame frame, double xMaxAbsoluteX, double yMaxAbsoluteY, double zMaxAbsoluteZ)
   {
      FramePoint randomPoint = new FramePoint(frame, RandomNumbers.nextDouble(random, xMaxAbsoluteX),
                                              RandomNumbers.nextDouble(random, yMaxAbsoluteY), RandomNumbers.nextDouble(random, zMaxAbsoluteZ));
      return randomPoint;
   }

   public static FramePoint generateRandomFramePoint(Random random, ReferenceFrame frame, double xMin, double xMax, double yMin, double yMax, double zMin,
                                                     double zMax)
   {
      FramePoint randomPoint = new FramePoint(frame, RandomNumbers.nextDouble(random, xMin, xMax), RandomNumbers.nextDouble(random, yMin, yMax),
                                              RandomNumbers.nextDouble(random, zMin, zMax));
      return randomPoint;
   }

   /**
    * Creates a new FramePoint2d based on the x and y components of this FramePoint
    */
   public FramePoint2d toFramePoint2d()
   {
      return new FramePoint2d(this.getReferenceFrame(), this.getX(), this.getY());
   }

   /**
    * Returns the Point3D used in this FramePoint
    *
    * @return Point3D
    */
   public Point3D getPoint()
   {
      return this.tuple;
   }
}
