package us.ihmc.robotics.geometry;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Vector3d;

public class FrameOrientation2d extends ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;
   private double yaw = 0.0;

   public FrameOrientation2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameOrientation2d(FrameOrientation2d orientation)
   {
      this.referenceFrame = orientation.referenceFrame;
      this.yaw = orientation.yaw;
   }

   public FrameOrientation2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 0.0);
   }

   public FrameOrientation2d(ReferenceFrame referenceFrame, double yaw)
   {
      handleNullFrame(referenceFrame);
      this.referenceFrame = referenceFrame;
      this.yaw = yaw;
   }

   public void handleNullFrame(ReferenceFrame referenceFrame)
   {
      if (referenceFrame == null)
         throw new RuntimeException("FrameOrientation2d::FrameOrientation2d: created a FrameOrientation2d with a null reference frame.");
   }

   public void interpolate(FrameOrientation2d orientationOne, FrameOrientation2d orientationTwo, double alpha)
   {
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);
      extrapolate(orientationOne, orientationTwo, alpha);
   }

   public void extrapolate(FrameOrientation2d orientationOne, FrameOrientation2d orientationTwo, double alpha)
   {
      orientationOne.checkReferenceFrameMatch(orientationTwo);

      double deltaYaw = AngleTools.computeAngleDifferenceMinusPiToPi(orientationTwo.yaw, orientationOne.yaw);
      this.yaw = AngleTools.trimAngleMinusPiToPi(orientationOne.yaw + alpha * deltaYaw);

      this.referenceFrame = orientationOne.getReferenceFrame();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void getTransform3D(RigidBodyTransform transformToPack)
   {
      transformToPack.setEuler(0.0, 0.0, this.yaw);
   }

   public double getYaw()
   {
      return this.yaw;
   }

   public void set(FrameOrientation2d orientation)
   {
      referenceFrame.checkReferenceFrameMatch(orientation.referenceFrame);
      this.yaw = orientation.yaw;
   }

   public void set(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      this.yaw = 0.0;
   }

   public void setIncludingFrame(FrameOrientation2d orientation)
   {
      referenceFrame = orientation.referenceFrame;
      this.yaw = orientation.yaw;
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double yaw)
   {
      this.referenceFrame = referenceFrame;
      setYaw(yaw);
   }

   public void setYaw(double yaw)
   {
      if (Double.isNaN(yaw))
      {
         throw new RuntimeException("Orientation.setYaw() yaw = " + yaw);
      }

      this.yaw = AngleTools.trimAngleMinusPiToPi(yaw);
   }

   @Override
   public String toString()
   {
      String stringToReturn = "";

      stringToReturn = stringToReturn + "yaw: (" + yaw + ")\n";

      stringToReturn = stringToReturn + "frame: " + referenceFrame;

      return stringToReturn;
   }

   private RigidBodyTransform temporaryTransformHToDesiredFrame;

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      if (temporaryTransformHToDesiredFrame == null)
         temporaryTransformHToDesiredFrame = new RigidBodyTransform();

      referenceFrame.getTransformToDesiredFrame(temporaryTransformHToDesiredFrame, desiredFrame);
      checkIsTransformationInPlane(temporaryTransformHToDesiredFrame);
      Vector3d xVector = new Vector3d(1.0, 0.0, 0.0);
      temporaryTransformHToDesiredFrame.transform(xVector);
      double deltaYaw = Math.atan2(xVector.getY(), xVector.getX());
      if (Double.isNaN(deltaYaw) || Double.isInfinite(deltaYaw))
         deltaYaw = 0.0;
      this.yaw = AngleTools.trimAngleMinusPiToPi(this.yaw + deltaYaw);
      this.referenceFrame = desiredFrame;
   }

   public boolean epsilonEquals(FrameOrientation2d orientation, double epsilon)
   {
      checkReferenceFrameMatch(orientation);

      return (Math.abs(this.yaw - orientation.yaw) < epsilon);
   }

   private void checkIsTransformationInPlane(RigidBodyTransform transform)
   {
      if (!ReferenceFrame.isTransformationInPlane(transform))
      {
         throw new RuntimeException("Cannot transform FramePoint2d to a plane with a different surface normal");
      }
   }

   public double sub(FrameOrientation2d orientationToSubtract)
   {
      return AngleTools.trimAngleMinusPiToPi(this.yaw - orientationToSubtract.yaw);
   }
}
