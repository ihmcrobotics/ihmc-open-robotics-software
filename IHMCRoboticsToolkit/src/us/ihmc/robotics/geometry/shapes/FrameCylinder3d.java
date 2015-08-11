package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class FrameCylinder3d extends FrameShape3d
{
   private ReferenceFrame referenceFrame;
   private Cylinder3d cylinder3d;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();

   public FrameCylinder3d(FrameCylinder3d other)
   {
      this(other.referenceFrame, other.cylinder3d);
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      this.cylinder3d = new Cylinder3d(1.0, 0.1);
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame, Cylinder3d Cylinder3d)
   {
      this.referenceFrame = referenceFrame;
      this.cylinder3d = new Cylinder3d(Cylinder3d);
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame, RigidBodyTransform configuration, double height, double radius)
   {
      this.referenceFrame = referenceFrame;
      this.cylinder3d = new Cylinder3d(configuration, height, radius);
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame, double height, double radius)
   {
      this.referenceFrame = referenceFrame;
      this.cylinder3d = new Cylinder3d(height, radius);
   }

   public Cylinder3d getCylinder3d()
   {
      return cylinder3d;
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
         cylinder3d.applyTransform(temporaryTransformToDesiredFrame);
         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   public double getRadius()
   {
      return cylinder3d.getRadius();
   }

   public double getHeight()
   {
      return cylinder3d.getHeight();
   }

   @Override
   public double distance(FramePoint point)
   {
      checkReferenceFrameMatch(point);

      return cylinder3d.distance(point.getPoint());
   }

   @Override
   public void orthogonalProjection(FramePoint point)
   {
      checkReferenceFrameMatch(point);
      cylinder3d.orthogonalProjection(point.getPoint());
   }

   @Override
   public void applyTransform(RigidBodyTransform transformation)
   {
      cylinder3d.applyTransform(transformation);
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToTest)
   {
      checkReferenceFrameMatch(pointToTest);

      return cylinder3d.isInsideOrOnSurface(pointToTest.getPoint());
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToTest, double epsilon)
   {
      checkReferenceFrameMatch(pointToTest);

      return cylinder3d.isInsideOrOnSurface(pointToTest.getPoint(), epsilon);
   }

   @Override
   public void getClosestPointAndNormalAt(FramePoint intersectionToPack, FrameVector normalToPack, FramePoint pointToCheck)
   { // Assumes the point is inside. Otherwise, it doesn't really matter.
      checkReferenceFrameMatch(pointToCheck);
      normalToPack.changeFrame(referenceFrame);
      intersectionToPack.changeFrame(referenceFrame);
      cylinder3d.checkIfInside(pointToCheck.getPoint(), intersectionToPack.getPoint(), normalToPack.getVector());
   }

   public void setAndChangeFrame(FrameCylinder3d other)
   {
      this.referenceFrame = other.referenceFrame;
      this.cylinder3d.set(other.cylinder3d);
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceFrame: " + referenceFrame + ")\n");
      builder.append(cylinder3d.toString());

      return builder.toString();
   }

   public boolean checkIfInside(FramePoint pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      checkReferenceFrameMatch(pointInWorldToCheck);
      
      return cylinder3d.checkIfInside(pointInWorldToCheck.getPoint(), closestPointToPack, normalToPack);
   }
   
}
