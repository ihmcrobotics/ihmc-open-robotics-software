package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3DReadOnly;
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
   public FramePoint(ReferenceFrame referenceFrame, Tuple3DReadOnly position)
   {
      super(referenceFrame, new Point3D(position));
   }

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint(ReferenceFrame referenceFrame, double[] position)
   {
      super(referenceFrame, new Point3D(position));
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
   public FramePoint(FrameTuple3D<?, ?> frameTuple)
   {
      super(frameTuple.getReferenceFrame(), new Point3D(frameTuple.tuple));
   }

   /**
    * FramePoint
    * <p/>
    * A normal point associated with a specific reference frame.
    */
   public FramePoint(FrameTuple2D<?, ?> frameTuple2d)
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

   public static FramePoint getMidPoint(FramePoint point1, FramePoint point2)
   {
      point1.checkReferenceFrameMatch(point2);
      FramePoint midPoint = new FramePoint(point1.referenceFrame);
      midPoint.interpolate(point1, point2, 0.5);
      return midPoint;
   }

   /**
    * yawAboutPoint
    *
    * @param pointToYawAbout FramePoint
    * @param yaw double
    * @return CartesianPositionFootstep
    */
   public void yawAboutPoint(FramePoint pointToYawAbout, FramePoint resultToPack, double yaw)
   {
      checkReferenceFrameMatch(pointToYawAbout);
      double tempX = getX() - pointToYawAbout.getX();
      double tempY = getY() - pointToYawAbout.getY();
      double tempZ = getZ() - pointToYawAbout.getZ();

      double cosAngle = Math.cos(yaw);
      double sinAngle = Math.sin(yaw);

      double x = cosAngle * tempX + -sinAngle * tempY;
      tempY = sinAngle * tempX + cosAngle * tempY;
      tempX = x;

      resultToPack.setIncludingFrame(pointToYawAbout);
      resultToPack.add(tempX, tempY, tempZ);
   }

   public void pitchAboutPoint(FramePoint pointToPitchAbout, FramePoint resultToPack, double pitch)
   {
      checkReferenceFrameMatch(pointToPitchAbout);
      double tempX = getX() - pointToPitchAbout.getX();
      double tempY = getY() - pointToPitchAbout.getY();
      double tempZ = getZ() - pointToPitchAbout.getZ();

      double cosAngle = Math.cos(pitch);
      double sinAngle = Math.sin(pitch);

      double x = cosAngle * tempX + sinAngle * tempZ;
      tempZ = -sinAngle * tempX + cosAngle * tempZ;
      tempX = x;

      resultToPack.setIncludingFrame(pointToPitchAbout);
      resultToPack.add(tempX, tempY, tempZ);
   }
}
