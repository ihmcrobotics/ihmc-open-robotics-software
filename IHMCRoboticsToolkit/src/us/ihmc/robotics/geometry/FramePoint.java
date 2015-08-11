package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import java.util.List;
import java.util.Random;

/**
 * One of the main goals of this class is to check, at runtime, that operations on points occur within the same Frame.
 * This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class FramePoint extends FrameTuple<Point3d>
{
   private static final long serialVersionUID = -4831948077397801540L;

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(ReferenceFrame referenceFrame, Tuple3d position)
   {
      this(referenceFrame, position, null);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(ReferenceFrame referenceFrame, Tuple3d position, String name)
   {
      super(referenceFrame, new Point3d(position), name);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(ReferenceFrame referenceFrame, double[] position)
   {
      this(referenceFrame, position, null);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(ReferenceFrame referenceFrame, double[] position, String name)
   {
      super(referenceFrame, new Point3d(position), name);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Point3d(), null);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(ReferenceFrame referenceFrame, String name)
   {
      super(referenceFrame, new Point3d(), name);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(FrameTuple<?> frameTuple)
   {
      super(frameTuple.referenceFrame, new Point3d(frameTuple.tuple), frameTuple.name);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(FrameTuple2d<?> frameTuple2d)
   {
      super(frameTuple2d.referenceFrame, new Point3d(), frameTuple2d.name);
      setXY(frameTuple2d);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this(referenceFrame, x, y, z, null);
   }

   /** FramePoint <p/> A normal point associated with a specific reference frame. */
   public FramePoint(ReferenceFrame referenceFrame, double x, double y, double z, String name)
   {
      super(referenceFrame, new Point3d(x, y, z), name);
   }

   public static FramePoint average(List<? extends FramePoint> framePoints)
   {
      ReferenceFrame frame = framePoints.get(0).getReferenceFrame();
      FramePoint nextPoint = new FramePoint(frame);

      FramePoint average = new FramePoint(framePoints.get(0));
      for (int i = 1; i < framePoints.size(); i++)
      {
         nextPoint.set(framePoints.get(i));
         nextPoint.changeFrame(frame);
         average.add(nextPoint);
      }

      average.scale(1.0 / ((double) framePoints.size()));
      return average;
   }

   public static FramePoint generateRandomFramePoint(Random random, ReferenceFrame frame, double xMaxAbsoluteX, double yMaxAbsoluteY, double zMaxAbsoluteZ)
   {
      FramePoint randomPoint = new FramePoint(frame, RandomTools.generateRandomDouble(random, xMaxAbsoluteX), RandomTools
            .generateRandomDouble(random, yMaxAbsoluteY),
            RandomTools.generateRandomDouble(random, zMaxAbsoluteZ));
      return randomPoint;
   }

   public static FramePoint generateRandomFramePoint(Random random, ReferenceFrame frame, double xMin, double xMax, double yMin, double yMax, double zMin,
         double zMax)
   {
      FramePoint randomPoint = new FramePoint(frame, RandomTools.generateRandomDouble(random, xMin, xMax), RandomTools.generateRandomDouble(random, yMin, yMax), RandomTools
            .generateRandomDouble(random, zMin, zMax));
      return randomPoint;
   }

   public double getXYPlaneDistance(FramePoint framePoint)
   {
      checkReferenceFrameMatch(framePoint);

      double dx, dy;

      dx = this.getX() - framePoint.getX();
      dy = this.getY() - framePoint.getY();
      return Math.sqrt(dx * dx + dy * dy);
   }

   public double getXYPlaneDistance(FramePoint2d framePoint2d)
   {
      checkReferenceFrameMatch(framePoint2d);

      double dx, dy;

      dx = this.getX() - framePoint2d.getX();
      dy = this.getY() - framePoint2d.getY();
      return Math.sqrt(dx * dx + dy * dy);
   }

   public double distance(Point3d point)
   {
      return this.tuple.distance(point);
   }

   public double distance(FramePoint framePoint)
   {
      checkReferenceFrameMatch(framePoint);

      return this.tuple.distance(framePoint.tuple);
   }

   public double distanceSquared(FramePoint framePoint)
   {
      checkReferenceFrameMatch(framePoint);

      return this.tuple.distanceSquared(framePoint.tuple);
   }

   /**
    * Creates a new FramePoint2d based on the x and y components of this FramePoint
    */
   public FramePoint2d toFramePoint2d()
   {
      return new FramePoint2d(this.getReferenceFrame(), this.getX(), this.getY());
   }
   
   /**
    * Makes the pointToPack a FramePoint2d version of this FramePoint
    * 
    * @param pointToPack
    */
   public void getFramePoint2d(FramePoint2d pointToPack)
   {
      pointToPack.setIncludingFrame(this.getReferenceFrame(), this.getX(), this.getY());
   }

   /**
    * Returns the Point3d used in this FramePoint
    *
    * @return Point3d
    */
   public Point3d getPoint()
   {
      return this.tuple;
   }

   /**
    * Changes frame of this FramePoint to the given ReferenceFrame, using the given Transform3D and returns a copy.
    *
    * @param desiredFrame        ReferenceFrame to change the FramePoint into.
    * @param transformToNewFrame Transform3D from the current frame to the new desiredFrame
    * @return Copied FramePoint in the new reference frame.
    */
   public FramePoint changeFrameUsingTransformCopy(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      FramePoint ret = new FramePoint(this);
      ret.changeFrameUsingTransform(desiredFrame, transformToNewFrame);
      return ret;
   }

   /**
    * Changes frame of this FramePoint to the given ReferenceFrame.
    *
    * @param desiredFrame ReferenceFrame to change the FramePoint into.
    */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame != referenceFrame)
      {
         referenceFrame.verifySameRoots(desiredFrame);
         RigidBodyTransform referenceTf, desiredTf;
         
         if ((referenceTf = referenceFrame.getTransformToRoot()) != null)
         {
           referenceTf.transform(tuple);
         }

         if ((desiredTf = desiredFrame.getInverseTransformToRoot()) != null)
         {
            desiredTf.transform(tuple);
         }
         
         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   /**
    * Changes frame of this FramePoint to the given ReferenceFrame, using the given Transform3D.
    *
    * @param desiredFrame        ReferenceFrame to change the FramePoint into.
    * @param transformToNewFrame Transform3D from the current frame to the new desiredFrame
    */
   @Override
   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      transformToNewFrame.transform(tuple);
      referenceFrame = desiredFrame;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(this.tuple);
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
    * @param yaw             double
    * @return CartesianPositionFootstep
    */
   public FramePoint yawAboutPoint(FramePoint pointToYawAbout, double yaw)
   {
      FrameVector v0 = new FrameVector(this);
      v0.sub(pointToYawAbout);

      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.rotZ(yaw);

      v0.applyTransform(transform3D);

      FramePoint ret = new FramePoint(pointToYawAbout);
      ret.add(v0);

      return ret;
   }

   public FramePoint pitchAboutPoint(FramePoint pointToPitchAbout, double pitch)
   {
      FrameVector v0 = new FrameVector(this);
      v0.sub(pointToPitchAbout);

      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.rotY(pitch);

      v0.applyTransform(transform3D);

      FramePoint ret = new FramePoint(pointToPitchAbout);
      ret.add(v0);

      return ret;
   }
}
