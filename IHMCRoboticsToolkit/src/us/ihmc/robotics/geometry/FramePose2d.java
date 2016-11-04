package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point2d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class FramePose2d extends AbstractReferenceFrameHolder implements FrameObject<FramePose2d>
{
   private final FramePoint2d position;
   private final FrameOrientation2d orientation;
   private ReferenceFrame referenceFrame;

   public FramePose2d()
   {
      this(new FramePoint2d(), new FrameOrientation2d());
   }

   public FramePose2d(ReferenceFrame referenceFrame)
   {
      this.position = new FramePoint2d(referenceFrame);
      this.orientation = new FrameOrientation2d(referenceFrame);
      this.referenceFrame = referenceFrame;
   }

   public FramePose2d(FramePoint2d position, FrameOrientation2d orientation)
   {
      if (position.getReferenceFrame() != orientation.getReferenceFrame())
      {
         throw new ReferenceFrameMismatchException("FramePose: The position frame (" + position.getReferenceFrame()
               + ") does not match the orientation frame (" + orientation.getReferenceFrame() + ")");
      }

      this.position = new FramePoint2d(position);
      this.orientation = new FrameOrientation2d(orientation);
      this.referenceFrame = position.getReferenceFrame();
   }

   public FramePose2d(ReferenceFrame referenceFrame, Point2d position, double orientation)
   {
      this.position = new FramePoint2d(referenceFrame, position);
      this.orientation = new FrameOrientation2d(referenceFrame, orientation);
      this.referenceFrame = referenceFrame;
   }

   public FramePose2d(FramePose2d framePose)
   {
      this.position = new FramePoint2d(framePose.position);
      this.orientation = new FrameOrientation2d(framePose.orientation);
      this.referenceFrame = framePose.referenceFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void getPose(RigidBodyTransform transformToPack)
   {
      orientation.getTransform3D(transformToPack);
      transformToPack.setTranslation(new Vector3d(position.getX(), position.getY(), 0.0));
   }

   public void set(FramePose2d pose)
   {
      position.set(pose.position);
      orientation.set(pose.orientation);
   }

   public void setIncludingFrame(FramePose2d pose)
   {
      this.referenceFrame = pose.getReferenceFrame();
      position.setIncludingFrame(referenceFrame, pose.getX(), pose.getY());
      orientation.setIncludingFrame(referenceFrame, pose.getYaw());
   }

   public void setOrientation(FrameOrientation2d orientation)
   {
      this.orientation.set(orientation);
   }

   public void setPosition(FramePoint2d position)
   {
      this.position.set(position);
   }

   private final Vector3d tempVector = new Vector3d();
   private final Quat4d tempQuat = new Quat4d();
   
   public void setPose(RigidBodyTransform transformToWorld)
   {
      transformToWorld.getTranslation(tempVector);
      transformToWorld.getRotation(tempQuat);

      setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), tempVector.getX(), tempVector.getY(), RotationTools.computeYaw(tempQuat));
   }
   
   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double yaw)
   {
      this.referenceFrame = referenceFrame;
      position.setIncludingFrame(referenceFrame, x, y);
      orientation.setIncludingFrame(referenceFrame, yaw);
   }
   
   public void setX(double x)
   {
      position.setX(x);
   }

   public void setY(double y)
   {
      position.setY(y);
   }
   
   public void setYaw(double yaw)
   {
      orientation.setIncludingFrame(referenceFrame, yaw);
   }

   public double getYaw()
   {
      return this.orientation.getYaw();
   }

   public double getX()
   {
      return this.position.getX();
   }

   public double getY()
   {
      return this.position.getY();
   }

   public void getPosition(FramePoint2d framePoint2dToPack)
   {
      framePoint2dToPack.setIncludingFrame(position);
   }

   public void getOrientation(FrameOrientation2d orientationToPack)
   {
      orientationToPack.setIncludingFrame(this.orientation);
   }

   public void interpolate(FramePose2d framePose1, FramePose2d framePose2, double alpha)
   {
      position.interpolate(framePose1.position, framePose2.position, alpha);
      orientation.interpolate(framePose1.orientation, framePose2.orientation, alpha);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == referenceFrame)
         return;

      position.changeFrame(desiredFrame);
      orientation.changeFrame(desiredFrame);
      referenceFrame = desiredFrame;
   }
   
   @Override
   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      position.changeFrameUsingTransform(desiredFrame, transformToNewFrame);
      orientation.changeFrameUsingTransform(desiredFrame, transformToNewFrame);
      
      referenceFrame = desiredFrame;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      position.applyTransform(transform);
      orientation.applyTransform(transform);
   }
   
   
   private final FramePoint2d otherPosition = new FramePoint2d();
   public double getPositionDistance(FramePose2d framePose)
   {
      checkReferenceFrameMatch(framePose);
      framePose.getPosition(otherPosition);
      
      return position.distance(otherPosition);
   }
   
   public double getOrientationDistance(FramePose2d framePose)
   {
      checkReferenceFrameMatch(framePose);
      
      RigidBodyTransform transformThis = new RigidBodyTransform();
      this.getPose(transformThis);
      
      RigidBodyTransform transformThat = new RigidBodyTransform();
      framePose.getPose(transformThat);
      
      return TransformTools.getMagnitudeOfAngleOfRotation(TransformTools.getTransformFromA2toA1(transformThis, transformThat));
   }

   @Override
   public boolean epsilonEquals(FramePose2d framePose, double epsilon)
   {
      if (!position.epsilonEquals(framePose.position, epsilon))
         return false;

      if (!orientation.epsilonEquals(framePose.orientation, epsilon))
         return false;

      return true;
   }

   @Override
   public void setToZero()
   {
      position.setToZero();
      orientation.setToZero();
   }

   @Override
   public void setToNaN()
   {
      position.setToNaN();
      orientation.setToNaN();      
   }

   @Override
   public boolean containsNaN()
   {
      return (position.containsNaN() || orientation.containsNaN());
   }

   @Override
   public String toString()
   {
      return position + ", " + orientation.toString();
   }

   @Override
   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      this.setToZero();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      this.setToNaN();
   }
}
