package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

//the torus is built in the XY plane, around the z axis
public class FrameTorus3d extends FrameShape3d
{
   private ReferenceFrame referenceFrame;
   private Torus3d torus3d;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();

   public FrameTorus3d(FrameTorus3d other)
   {
      this(other.referenceFrame, other.torus3d);
   }

   public FrameTorus3d(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      this.torus3d = new Torus3d(1.0, 0.1);
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, Torus3d Torus3d)
   {
      this.referenceFrame = referenceFrame;
      this.torus3d = new Torus3d(Torus3d);
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, RigidBodyTransform configuration, double radius, double thickness)
   {
      this.referenceFrame = referenceFrame;
      this.torus3d = new Torus3d(configuration, radius, thickness);
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, double radius, double thickness)
   {
      this.referenceFrame = referenceFrame;
      this.torus3d = new Torus3d(radius, thickness);
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, FramePose pose, double radius, double thickness)
   {
      this.referenceFrame = referenceFrame;
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      pose.getPose(rigidBodyTransform);
      this.torus3d = new Torus3d(rigidBodyTransform, radius, thickness);
   }

   public Torus3d getTorus3d()
   {
      return torus3d;
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
         torus3d.applyTransform(temporaryTransformToDesiredFrame);
         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   public double getRadius()
   {
      return torus3d.getRadius();
   }

   public double getThickness()
   {
      return torus3d.getThickness();
   }

   @Override
   public double distance(FramePoint point)
   {
      checkReferenceFrameMatch(point);

      return torus3d.distance(point.getPoint());
   }

   @Override
   public void orthogonalProjection(FramePoint point)
   {
      checkReferenceFrameMatch(point);
      torus3d.orthogonalProjection(point.getPoint());
   }

   @Override
   public void applyTransform(RigidBodyTransform transformation)
   {
      torus3d.applyTransform(transformation);
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToTest)
   {
      checkReferenceFrameMatch(pointToTest);

      return torus3d.isInsideOrOnSurface(pointToTest.getPoint());
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToTest, double epsilon)
   {
      checkReferenceFrameMatch(pointToTest);

      return torus3d.isInsideOrOnSurface(pointToTest.getPoint(), epsilon);
   }

   @Override
   public void getClosestPointAndNormalAt(FramePoint intersectionToPack, FrameVector normalToPack, FramePoint pointToCheck)
   { // Assumes the point is inside. Otherwise, it doesn't really matter.
      checkReferenceFrameMatch(pointToCheck);
      normalToPack.changeFrame(referenceFrame);
      intersectionToPack.changeFrame(referenceFrame);
      torus3d.checkIfInside(pointToCheck.getPoint(), intersectionToPack.getPoint(), normalToPack.getVector());
   }

   public void setAndChangeFrame(FrameTorus3d other)
   {
      this.referenceFrame = other.referenceFrame;
      this.torus3d.set(other.torus3d);
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceFrame: " + referenceFrame + ")\n");
      builder.append(torus3d.toString());

      return builder.toString();
   }

   public boolean checkIfInside(FramePoint pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      checkReferenceFrameMatch(pointInWorldToCheck);
      
      return torus3d.checkIfInside(pointInWorldToCheck.getPoint(), closestPointToPack, normalToPack);
   }

}
