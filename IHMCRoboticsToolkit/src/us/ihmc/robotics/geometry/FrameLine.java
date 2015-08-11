package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

public class FrameLine extends ReferenceFrameHolder
{
   private static final Vector3d zero = new Vector3d(0.0, 0.0, 0.0);

   private ReferenceFrame referenceFrame;

   private final Point3d origin = new Point3d();
   private final Vector3d direction = new Vector3d();

   private RigidBodyTransform temporaryTransformToDesiredFrame;

   public FrameLine(FramePoint origin, FrameVector direction)
   {
      origin.checkReferenceFrameMatch(direction);
      checkDirectionValidity(direction.getVector());

      origin.get(this.origin);
      direction.get(this.direction);
      this.direction.normalize();

      referenceFrame = origin.getReferenceFrame();
   }

   public FrameLine(ReferenceFrame referenceFrame, Tuple3d origin, Tuple3d direction)
   {
      checkDirectionValidity(direction);

      this.referenceFrame = referenceFrame;
      this.origin.set(origin);
      this.direction.set(direction);
      this.direction.normalize();
   }

   public FrameLine(FrameLine frameLine)
   {
      checkDirectionValidity(frameLine.direction);

      this.referenceFrame = frameLine.referenceFrame;
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
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame != referenceFrame)
      {
         if (temporaryTransformToDesiredFrame == null)
            temporaryTransformToDesiredFrame = new RigidBodyTransform();

         referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
         temporaryTransformToDesiredFrame.transform(origin);
         temporaryTransformToDesiredFrame.transform(direction);
         referenceFrame = desiredFrame;
      }
   }

   public boolean epsilonEquals(FrameLine otherLine, double epsilon)
   {
      checkReferenceFrameMatch(otherLine);

      return origin.epsilonEquals(otherLine.origin, epsilon) && direction.epsilonEquals(otherLine.direction, epsilon);
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
