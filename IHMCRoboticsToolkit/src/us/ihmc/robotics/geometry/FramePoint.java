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
    * Creates a new frame point and initializes it components to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FramePoint()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame point and initializes it components to zero and its reference frame to the
    * {@code referenceFrame}.
    * 
    * @param referenceFrame the initial frame for this frame point.
    */
   public FramePoint(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Point3D());
   }

   /**
    * Creates a new frame point and initializes it with the given components and the given reference
    * frame.
    * 
    * @param referenceFrame the initial frame for this frame point.
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    */
   public FramePoint(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(referenceFrame, new Point3D(x, y, z));
   }

   /**
    * Creates a new frame point and initializes its component {@code x}, {@code y}, {@code z} in
    * order from the given array and initializes its reference frame.
    * 
    * @param referenceFrame the initial frame for this frame point.
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public FramePoint(ReferenceFrame referenceFrame, double[] pointArray)
   {
      super(referenceFrame, new Point3D(pointArray));
   }

   /**
    * Creates a new frame point and initializes it to {@code tuple3DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame point.
    * @param tuple3DReadOnly the tuple to copy the components from. Not modified.
    */
   public FramePoint(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      super(referenceFrame, new Point3D(tuple3DReadOnly));
   }

   /**
    * Creates a new frame point and initializes its reference frame x and y coordinates from
    * {@code frameTuple2DReadOnly}.
    *
    * @param frameTuple2DReadOnly the tuple to copy the components and reference frame from. Not
    *           modified.
    */
   public FramePoint(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(frameTuple2DReadOnly.getReferenceFrame(), new Point3D(frameTuple2DReadOnly));
   }

   /**
    * Creates a new frame point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FramePoint(FrameTuple3DReadOnly other)
   {
      super(other.getReferenceFrame(), new Point3D(other));
   }

   public static FramePoint generateRandomFramePoint(Random random, ReferenceFrame frame, double xMaxAbsoluteX, double yMaxAbsoluteY, double zMaxAbsoluteZ)
   {
      FramePoint randomPoint = new FramePoint(frame, RandomNumbers.nextDouble(random, xMaxAbsoluteX), RandomNumbers.nextDouble(random, yMaxAbsoluteY),
                                              RandomNumbers.nextDouble(random, zMaxAbsoluteZ));
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
    * Gets the read-only reference to the point used in {@code this}.
    *
    * @return the point of {@code this}.
    */
   public Point3D getPoint()
   {
      return tuple;
   }
}
