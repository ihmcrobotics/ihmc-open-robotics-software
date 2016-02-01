package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEllipsoid3d extends FrameShape3d
{
   private ReferenceFrame referenceFrame;
   private final Ellipsoid3d ellipsoid;

   public FrameEllipsoid3d(FrameEllipsoid3d other)
   {
      this(other.referenceFrame, other.ellipsoid);
   }

   public FrameEllipsoid3d(ReferenceFrame referenceFrame, Ellipsoid3d ellipsoid)
   {
      this.referenceFrame = referenceFrame;
      this.ellipsoid = new Ellipsoid3d(ellipsoid);
   }

   public FrameEllipsoid3d(ReferenceFrame referenceFrame, double xRadius, double yRadius, double zRadius, RigidBodyTransform transform)
   {
      this.referenceFrame = referenceFrame;
      this.ellipsoid = new Ellipsoid3d(xRadius, yRadius, zRadius, transform);
   }

   public FrameEllipsoid3d(ReferenceFrame referenceFrame, double xRadius, double yRadius, double zRadius)
   {
      this.referenceFrame = referenceFrame;
      this.ellipsoid = new Ellipsoid3d(xRadius, yRadius, zRadius);
   }

   public Ellipsoid3d getEllipsoid3d()
   {
      return ellipsoid;
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
         RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();

         referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
         ellipsoid.applyTransform(temporaryTransformToDesiredFrame);
         referenceFrame = desiredFrame;
      }
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      ellipsoid.applyTransform(transform);
   }

   @Override
   public double distance(FramePoint point)
   {
      checkReferenceFrameMatch(point);

      return ellipsoid.distance(point.getPoint());
   }

   @Override
   public void getClosestPointAndNormalAt(FramePoint closestPointToPack, FrameVector normalToPack, FramePoint pointInWorldToCheck)
   {
      checkReferenceFrameMatch(pointInWorldToCheck);
      normalToPack.changeFrame(referenceFrame);
      closestPointToPack.changeFrame(referenceFrame);
      ellipsoid.checkIfInside(pointInWorldToCheck.getPoint(), closestPointToPack.getPoint(), normalToPack.getVector());
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToCheck)
   {
      checkReferenceFrameMatch(pointToCheck);

      return ellipsoid.isInsideOrOnSurface(pointToCheck.getPoint());
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToCheck, double epsilon)
   {
      checkReferenceFrameMatch(pointToCheck);

      return ellipsoid.isInsideOrOnSurface(pointToCheck.getPoint(), epsilon);
   }

   @Override
   public void orthogonalProjection(FramePoint pointToCheckAndPack)
   {
      checkReferenceFrameMatch(pointToCheckAndPack);

      ellipsoid.orthogonalProjection(pointToCheckAndPack.getPoint());
   }

}
