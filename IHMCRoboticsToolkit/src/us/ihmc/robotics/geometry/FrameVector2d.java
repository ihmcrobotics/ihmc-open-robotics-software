package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * One of the main goals of this class is to check, at runtime, that operations on vectors occur
 * within the same Frame. This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class FrameVector2d extends FrameTuple2d<FrameVector2d, Vector2D>
{
   private static final long serialVersionUID = -610124454205790361L;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(ReferenceFrame referenceFrame, double x, double y, String name)
   {
      super(referenceFrame, new Vector2D(x, y), name);
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(ReferenceFrame referenceFrame, double x, double y)
   {
      this(referenceFrame, x, y, null);
   }

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
   public FrameVector2d(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple)
   {
      this(referenceFrame, tuple.getX(), tuple.getY());
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(ReferenceFrame referenceFrame, double[] vector)
   {
      this(referenceFrame, vector[0], vector[1]);
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 0.0, 0.0);
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(FrameTuple2d<?, ?> frameTuple2d)
   {
      this(frameTuple2d.referenceFrame, frameTuple2d.tuple.getX(), frameTuple2d.tuple.getY(), frameTuple2d.name);
   }

   /**
    * FrameVector2d
    * <p/>
    * A normal vector2d associated with a specific reference frame.
    */
   public FrameVector2d(FramePoint2d startFramePoint, FramePoint2d endFramePoint)
   {
      this(endFramePoint.referenceFrame, endFramePoint.tuple.getX(), endFramePoint.tuple.getY(), endFramePoint.name);
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
   public Vector2D getVector()
   {
      return this.tuple;
   }

   @Override
   public boolean equals(Object frameVector)
   {
      if (frameVector instanceof FrameVector2d)
      {
         boolean referenceFrameMatches = referenceFrame.equals(((FrameVector2d) frameVector).getReferenceFrame());
         boolean pointEquals = tuple.equals(((FrameVector2d) frameVector).tuple);
         return referenceFrameMatches && pointEquals;
      }
      else
      {
         return super.equals(frameVector);
      }
   }

   public boolean eplilonEquals(FrameVector2d frameVector, double epsilon)
   {
      boolean referenceFrameMatches = referenceFrame.equals(frameVector.getReferenceFrame());
      boolean pointEquals = tuple.epsilonEquals(frameVector.tuple, epsilon);
      return referenceFrameMatches && pointEquals;
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

      return this.tuple.getX() * frameVector.tuple.getY() - tuple.getY() * frameVector.tuple.getX();
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

   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      getGeometryObject().applyTransform(temporaryTransformToDesiredFrame, false);
      this.referenceFrame = desiredFrame;
   }
}
