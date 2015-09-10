package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.random.RandomTools;

/**
 * <p>Title: FramePoint2d</p>
 *
 * <p>Description: A FramePoint2d is a normal point associated with a specified reference frame</p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: IHMC</p>
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class FramePoint2d extends FrameTuple2d<Point2d>
{
   private static final long serialVersionUID = -1287148635726098768L;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();
   private final Point3d temporaryTransformedPoint = new Point3d();
   private final Vector3d temporaryTranslation = new Vector3d();

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(ReferenceFrame referenceFrame, double x, double y, String name)
   {
      super(referenceFrame, new Point2d(x, y), name);
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(FrameTuple2d<?> frameTuple2d)
   {
      this(frameTuple2d.referenceFrame, frameTuple2d.tuple.x, frameTuple2d.tuple.y, frameTuple2d.name);
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 0.0, 0.0, null);
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(ReferenceFrame referenceFrame, double x, double y)
   {
      this(referenceFrame, x, y, null);
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(ReferenceFrame referenceFrame, double[] position)
   {
      this(referenceFrame, position[0], position[1], null);
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(ReferenceFrame referenceFrame, double[] position, String name)
   {
      this(referenceFrame, position[0], position[1], name);
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(ReferenceFrame referenceFrame, String name)
   {
      this(referenceFrame, 0.0, 0.0, name);
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(ReferenceFrame referenceFrame, Tuple2d position)
   {
      this(referenceFrame, position.getX(), position.getY(), null);
   }

   /** FramePoint2d <p/> A normal point2d associated with a specific reference frame. */
   public FramePoint2d(ReferenceFrame referenceFrame, Tuple2d position, String name)
   {
      this(referenceFrame, position.getX(), position.getY(), name);
   }

   public static FramePoint2d generateRandomFramePoint2d(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax)
   {
      FramePoint2d randomPoint = new FramePoint2d(zUpFrame, RandomTools.generateRandomDouble(random, xMin, xMax), RandomTools
            .generateRandomDouble(random, yMin, yMax));

      return randomPoint;
   }

   public double distance(FramePoint2d framePoint)
   {
      checkReferenceFrameMatch(framePoint);

      return this.tuple.distance(framePoint.tuple);
   }

   public double distanceSquared(FramePoint2d framePoint)
   {
      checkReferenceFrameMatch(framePoint);

      return this.tuple.distanceSquared(framePoint.tuple);
   }

   /**
    * Returns the point in this FramePoint2d.
    *
    * @return Point2d
    */
   public Point2d getPoint()
   {
      return tuple;
   }

   /**
    * Creates a new FramePoint based on the x and y components of this FramePoint2d
    */
   public FramePoint toFramePoint()
   {
      return new FramePoint(this.getReferenceFrame(), this.getX(), this.getY(), 0.0);
   }

//   public static FramePoint2d morph(FramePoint2d point1, FramePoint2d point2, double alpha)
//   {
//      FramePoint2d ret = new FramePoint2d();
//      ret.interpolate(point1, point2, alpha);
//      return ret;
//   }

   public void applyTransform(RigidBodyTransform transform, boolean requireTransformInPlane)
   {
      temporaryTransformedPoint.set(tuple.x, tuple.y, 0.0);
      transform.transform(temporaryTransformedPoint);

      if (requireTransformInPlane)
         checkIsTransformationInPlane(transform, temporaryTransformedPoint);

      this.tuple.set(temporaryTransformedPoint.x, temporaryTransformedPoint.y);
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      applyTransform(transform, true);
   }

   public FramePoint2d applyTransformCopy(RigidBodyTransform transform3D)
   {
      FramePoint2d ret = new FramePoint2d(this);
      ret.applyTransform(transform3D);
      return ret;
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      applyTransform(temporaryTransformToDesiredFrame);
      this.referenceFrame = desiredFrame;
   }

   /**
    * Changes frame of this FramePoint2d to the given ReferenceFrame, projects into xy plane.
    *
    * @param desiredFrame ReferenceFrame to change the FramePoint2d into.
    */
   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      applyTransform(temporaryTransformToDesiredFrame, false);
      this.referenceFrame = desiredFrame;
   }

   /**
    * Changes frame of this FramePoint2d to the given ReferenceFrame, projects into xy plane, and returns a copy.
    *
    * @param desiredFrame ReferenceFrame to change the FramePoint2d into.
    * @return Copied FramePoint2d in the new reference frame.
    */
   public FramePoint2d changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame)
   {
      FramePoint2d ret = new FramePoint2d(this);
      ret.changeFrameAndProjectToXYPlane(desiredFrame);
      return ret;
   }

   /**
    * Changes frame of this FramePoint2d to the given ReferenceFrame, using the given Transform3D and returns a copy.
    *
    * @param desiredFrame ReferenceFrame to change the FramePoint2d into.
    * @param transformToNewFrame Transform3D from the current frame to the new desiredFrame
    * @return Copied FramePoint2d in the new reference frame.
    */
   public FramePoint2d changeFrameUsingTransformCopy(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      FramePoint2d ret = new FramePoint2d(this);
      ret.changeFrameUsingTransform(desiredFrame, transformToNewFrame);
      return ret;
   }

   @Override
   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      applyTransform(transformToNewFrame);
      referenceFrame = desiredFrame;
   }

   /**
    * yawAboutPoint
    *
    * @param pointToYawAbout FramePoint2d
    * @param yaw double
    * @return CartesianPositionFootstep
    */
   public FramePoint2d yawAboutPoint(FramePoint2d pointToYawAbout, double yaw)
   {
      FrameVector2d v0 = new FrameVector2d(this);
      v0.sub(pointToYawAbout);

      temporaryTransformToDesiredFrame.setIdentity();
      temporaryTransformToDesiredFrame.rotZ(yaw);

      v0.applyTransform(temporaryTransformToDesiredFrame);

      FramePoint2d ret = new FramePoint2d(pointToYawAbout);
      ret.add(v0);

      return ret;
   }

   private void checkIsTransformationInPlane(RigidBodyTransform transformToNewFrame, Point3d transformedPoint)
   {
      transformToNewFrame.get(temporaryTranslation);
      if (Math.abs(temporaryTranslation.z - transformedPoint.z) > epsilon)
         throw new RuntimeException("Cannot transform FramePoint2d to a plane with a different surface normal");
   }
}
