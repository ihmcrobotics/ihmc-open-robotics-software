package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameRamp3d extends FrameShape3d
{
   private ReferenceFrame referenceFrame;
   private Ramp3d ramp3d;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();

   public Ramp3d getRamp3d()
   {
      return ramp3d;
   }

   public void getTransformCopy(RigidBodyTransform transformToPack)
   {
      transformToPack.set(this.ramp3d.getTransform());
   }

   public RigidBodyTransform getTransform()
   {
      return ramp3d.getTransform();
   }

   public void setTransform(RigidBodyTransform transform3D)
   {
      ramp3d.setTransform(transform3D);
   }

   public FrameRamp3d(FrameRamp3d other)
   {
      this(other.referenceFrame, other.ramp3d);
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, Ramp3d ramp3d)
   {
      this.referenceFrame = referenceFrame;
      this.ramp3d = new Ramp3d(ramp3d);
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, double width, double length, double height)
   {
      this.referenceFrame = referenceFrame;
      this.ramp3d = new Ramp3d(width, length, height);
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, RigidBodyTransform configuration, double width, double length, double height)
   {
      this.referenceFrame = referenceFrame;
      this.ramp3d = new Ramp3d(configuration, width, length, height);
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
         referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
         ramp3d.applyTransform(temporaryTransformToDesiredFrame);
         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   @Override
   public void applyTransform(RigidBodyTransform transformation)
   {
      ramp3d.applyTransform(transformation);
   }

   @Override
   public double distance(FramePoint point)
   {
      checkReferenceFrameMatch(point);

      return ramp3d.distance(point.getPoint());
   }

   @Override
   public void getClosestPointAndNormalAt(FramePoint intersectionToPack, FrameVector normalToPack, FramePoint pointToCheck)
   { // Assumes the point is inside. Otherwise, it doesn't really matter.
      checkReferenceFrameMatch(pointToCheck);
      normalToPack.changeFrame(referenceFrame);
      intersectionToPack.changeFrame(referenceFrame);
      ramp3d.checkIfInside(pointToCheck.getPoint(), intersectionToPack.getPoint(), normalToPack.getVector());
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToTest, double epsilon)
   {
      checkReferenceFrameMatch(pointToTest);

      return ramp3d.isInsideOrOnSurface(pointToTest.getPoint(), epsilon);
   }

   @Override
   public void orthogonalProjection(FramePoint point)
   {
      checkReferenceFrameMatch(point);
      ramp3d.orthogonalProjection(point.getPoint());
   }

   public void setAndChangeFrame(FrameRamp3d other)
   {
      this.referenceFrame = other.referenceFrame;
      this.ramp3d.set(other.ramp3d);
   }

   public void packFramePose(FramePose framePoseToPack)
   {
      framePoseToPack.setPoseIncludingFrame(referenceFrame, ramp3d.transform);
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceFrame: " + referenceFrame + ")\n");
      builder.append(ramp3d.toString());

      return builder.toString();
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToTest)
   {
      checkReferenceFrameMatch(pointToTest);

      return ramp3d.isInsideOrOnSurface(pointToTest.getPoint());
   }

}
