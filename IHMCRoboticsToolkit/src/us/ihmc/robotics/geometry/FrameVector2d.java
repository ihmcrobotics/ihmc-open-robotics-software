package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import java.util.Random;

/**
 * One of the main goals of this class is to check, at runtime, that operations on vectors occur within the same Frame.
 * This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class FrameVector2d extends FrameTuple2d<Vector2d>
{
   private static final long serialVersionUID = -610124454205790361L;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();
   private final Vector3d temporaryTransformedVector = new Vector3d();

   /** FrameVector2d <p/> A normal vector2d associated with a specific reference frame. */
   public FrameVector2d(ReferenceFrame referenceFrame, double x, double y, String name)
   {
      super(referenceFrame, new Vector2d(x, y), name);
   }

   /** FrameVector2d <p/> A normal vector2d associated with a specific reference frame. */
   public FrameVector2d(ReferenceFrame referenceFrame, double x, double y)
   {
      this(referenceFrame, x, y, null);
   }

   /** FrameVector2d <p/> A normal vector2d associated with a specific reference frame. */
   public FrameVector2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /** FrameVector2d <p/> A normal vector2d associated with a specific reference frame. */
   public FrameVector2d(ReferenceFrame referenceFrame, Tuple2d tuple)
   {
      this(referenceFrame, tuple.x, tuple.y);
   }

   /** FrameVector2d <p/> A normal vector2d associated with a specific reference frame. */
   public FrameVector2d(ReferenceFrame referenceFrame, double[] vector)
   {
      this(referenceFrame, vector[0], vector[1]);
   }

   /** FrameVector2d <p/> A normal vector2d associated with a specific reference frame. */
   public FrameVector2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 0.0, 0.0);
   }

   /** FrameVector2d <p/> A normal vector2d associated with a specific reference frame. */
   public FrameVector2d(FrameTuple2d<?> frameTuple2d)
   {
      this(frameTuple2d.referenceFrame, frameTuple2d.tuple.x, frameTuple2d.tuple.y, frameTuple2d.name);
   }

   /** FrameVector2d <p/> A normal vector2d associated with a specific reference frame. */
   public FrameVector2d(FramePoint2d startFramePoint, FramePoint2d endFramePoint)
   {
      this(endFramePoint.referenceFrame, endFramePoint.tuple.x, endFramePoint.tuple.y, endFramePoint.name);
      startFramePoint.checkReferenceFrameMatch(endFramePoint);
      sub(startFramePoint);
   }

   public static FrameVector2d generateRandomFrameVector2d(Random random, ReferenceFrame zUpFrame)
   {
      double randomAngle = RandomTools.generateRandomDouble(random, -Math.PI, Math.PI);

      FrameVector2d randomVector = new FrameVector2d(zUpFrame, Math.cos(randomAngle), Math.sin(randomAngle));

      return randomVector;
   }

   /**
    * Returns the vector inside this FrameVector.
    *
    * @return Vector2d
    */
   public Vector2d getVector()
   {
      return this.tuple;
   }

   public void rotate90()
   {
      double x = -tuple.getY();
      double y = tuple.getX();

      tuple.set(x, y);
   }

   public double dot(FrameVector2d frameVector)
   {
      checkReferenceFrameMatch(frameVector);

      return this.tuple.dot(frameVector.tuple);
   }

   public double cross(FrameVector2d frameVector)
   {
      checkReferenceFrameMatch(frameVector);

      return this.tuple.x * frameVector.tuple.y - tuple.y * frameVector.tuple.x;
   }

   public double angle(FrameVector2d frameVector)
   {
      checkReferenceFrameMatch(frameVector);

      return this.tuple.angle(frameVector.tuple);
   }

   public void normalize()
   {
      this.tuple.normalize();
   }

   public double length()
   {
      return tuple.length();
   }

   public double lengthSquared()
   {
      return tuple.lengthSquared();
   }

   public void clipMaxLength(double maxLength)
   {
      if (maxLength < 1e-7)
      {
         this.set(0.0, 0.0);
         return;
      }

      double length = this.length();
      if (length < maxLength)
         return;

      scale(maxLength / length);
   }

   /**
    * Changes frame of this FrameVector2d to the given ReferenceFrame.
    *
    * @param desiredFrame ReferenceFrame to change the FrameVector2d into.
    */
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      applyTransform(temporaryTransformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      temporaryTransformedVector.set(tuple.x, tuple.y, 0.0);
      transform.transform(temporaryTransformedVector);

      // Check if the transformation is planar:
      checkIsTransformationInPlane(transform, temporaryTransformedVector);

      this.tuple.set(temporaryTransformedVector.x, temporaryTransformedVector.y);
   }

   public FrameVector2d applyTransformCopy(RigidBodyTransform transform3D)
   {
      FrameVector2d ret = new FrameVector2d(this);
      ret.applyTransform(transform3D);
      return ret;
   }

   @Override
   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      applyTransform(transformToNewFrame);
      referenceFrame = desiredFrame;
   }

   @Override
   public FrameVector2d changeFrameUsingTransformCopy(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      FrameVector2d ret = new FrameVector2d(this);
      ret.changeFrameUsingTransform(desiredFrame, transformToNewFrame);
      return ret;
   }

   private void checkIsTransformationInPlane(RigidBodyTransform transformToNewFrame, Vector3d transformedVector)
   {
      if (Math.abs(transformedVector.z) > epsilon)
         throw new RuntimeException("Cannot transform FramePoint2d to a plane with a different surface normal");
   }
}
