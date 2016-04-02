package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.transformables.TransformableLine3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameLine extends AbstractFrameObject<FrameLine, TransformableLine3d>
{
   private static final Vector3d zero = new Vector3d(0.0, 0.0, 0.0);

   private final Point3d origin;
   private final Vector3d direction;

   public FrameLine(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new TransformableLine3d());

      origin = getGeometryObject().getOrigin();
      direction = getGeometryObject().getDirection();
   }

   public FrameLine(FramePoint origin, FrameVector direction)
   {
      this(origin.getReferenceFrame());

      origin.checkReferenceFrameMatch(direction);
      checkDirectionValidity(direction.getVector());

      origin.get(this.origin);
      direction.get(this.direction);
      this.direction.normalize();
   }

   public FrameLine(ReferenceFrame referenceFrame, Tuple3d origin, Tuple3d direction)
   {
      this(referenceFrame);

      checkDirectionValidity(direction);

      this.referenceFrame = referenceFrame;
      this.origin.set(origin);
      this.direction.set(direction);
      this.direction.normalize();
   }

   public FrameLine(FrameLine frameLine)
   {
      this(frameLine.getReferenceFrame());

      checkDirectionValidity(frameLine.direction);

      this.origin.set(frameLine.origin);
      this.direction.set(frameLine.direction);
      this.direction.normalize();
   }

   public FramePoint getFrameOrigin()
   {
      FramePoint ret = new FramePoint(referenceFrame, origin);
      return ret;
   }

   public FramePoint getOriginInFrame(ReferenceFrame desiredFrame)
   {
      FramePoint ret = getFrameOrigin();
      ret.changeFrame(desiredFrame);
      return ret;
   }

   public FrameVector getFrameDirection()
   {
      FrameVector ret = new FrameVector(referenceFrame, direction);
      return ret;
   }

   public FrameVector getDirectionInFrame(ReferenceFrame desiredFrame)
   {
      FrameVector ret = getFrameDirection();
      ret.changeFrame(desiredFrame);
      return ret;
   }

   public Point3d getOrigin()
   {
      return origin;
   }

   public Vector3d getDirection()
   {
      return direction;
   }

   public Point3d getOriginCopy()
   {
      return new Point3d(origin);
   }

   public Vector3d getDirectionCopy()
   {
      return new Vector3d(direction);
   }

   @Override
   public boolean epsilonEquals(FrameLine otherLine, double epsilon)
   {
      checkReferenceFrameMatch(otherLine);

      return origin.epsilonEquals(otherLine.origin, epsilon) && direction.epsilonEquals(otherLine.direction, epsilon);
   }
   
   public void setFromTwoPoints(FramePoint point1, FramePoint point2)
   {
      checkReferenceFrameMatch(point1);
      checkReferenceFrameMatch(point2);
      
      origin.set(point1.getPoint());
      direction.sub(point2.getPoint(), point1.getPoint());
      direction.normalize();
   }

   public void setOrigin(FramePoint origin)
   {
      checkReferenceFrameMatch(origin);

      this.origin.set(origin.getPoint());
   }

   public void setDirection(FrameVector direction)
   {
      checkReferenceFrameMatch(direction);
      checkDirectionValidity(direction.getVector());

      this.direction.set(direction.getVector());
   }

   private static void checkDirectionValidity(Tuple3d direction)
   {
      if (direction.epsilonEquals(zero, 1e-12))
      {
         throw new RuntimeException("Direction cannot be the zero vector");
      }
   }

}
