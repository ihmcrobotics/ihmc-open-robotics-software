package us.ihmc.robotics.robotDescription.collisionMeshDefinitionData;

public class SphereCollisionMeshDefinitionData extends CollisionMeshDefinitionData
{
   private double radius = 0.0;

   public SphereCollisionMeshDefinitionData(String parentJointName)
   {
      super(parentJointName);
   }

   public SphereCollisionMeshDefinitionData(String parentJointName, double radius)
   {
      super(parentJointName);
      this.radius = radius;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public double getRadius()
   {
      return radius;
   }
}