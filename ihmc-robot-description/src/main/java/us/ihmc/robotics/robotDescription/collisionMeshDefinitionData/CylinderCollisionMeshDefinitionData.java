package us.ihmc.robotics.robotDescription.collisionMeshDefinitionData;

public class CylinderCollisionMeshDefinitionData extends CollisionMeshDefinitionData
{
   private double radius = 0.0;
   private double height = 0.0;

   public CylinderCollisionMeshDefinitionData(String parentJointName)
   {
      super(parentJointName);
   }

   public CylinderCollisionMeshDefinitionData(String parentJointName, double radius, double height)
   {
      super(parentJointName);
      this.radius = radius;
      this.height = height;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public void setHeight(double height)
   {
      this.height = height;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }
}