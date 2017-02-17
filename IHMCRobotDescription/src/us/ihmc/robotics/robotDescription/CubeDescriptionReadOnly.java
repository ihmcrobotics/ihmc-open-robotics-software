package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class CubeDescriptionReadOnly implements ConvexShapeDescription
{
   private final double lengthX, widthY, heightZ;
   private final RigidBodyTransform rigidBodyTransformToCenter;

   public CubeDescriptionReadOnly(double lengthX, double widthY, double heightZ, RigidBodyTransform rigidBodyTransform)
   {
      this.lengthX = lengthX;
      this.widthY = widthY;
      this.heightZ = heightZ;
      this.rigidBodyTransformToCenter = new RigidBodyTransform(rigidBodyTransform);
   }

   public double getLengthX()
   {
      return lengthX;
   }

   public double getWidthY()
   {
      return widthY;
   }

   public double getHeightZ()
   {
      return heightZ;
   }

   public void getRigidBodyTransformToCenter(RigidBodyTransform transformToPack)
   {
      transformToPack.set(rigidBodyTransformToCenter);
   }

}
