package us.ihmc.robotics.robotDescription.collisionMeshDefinitionData;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public abstract class CollisionMeshDefinitionData
{
   private String parentJointName;

   private RigidBodyTransform transformToParentJoint = new RigidBodyTransform();

   private AppearanceDefinition yoAppearance = YoAppearance.Beige();

   private int collisionGroup = 0xffffffff;
   private int collisionMask = 0xffffffff;

   public CollisionMeshDefinitionData(String parentJointName)
   {
      this.parentJointName = parentJointName;
   }

   public String getParentJointName()
   {
      return parentJointName;
   }

   public RigidBodyTransform getTransformToParentJoint()
   {
      return transformToParentJoint;
   }

   public AppearanceDefinition getYoAppearance()
   {
      return yoAppearance;
   }

   public int getCollisionGroup()
   {
      return collisionGroup;
   }

   public int getCollisionMask()
   {
      return collisionMask;
   }

   public void setParentJointName(String parentJointName)
   {
      this.parentJointName = parentJointName;
   }

   public void setTransformToParentJoint(RigidBodyTransform transformToParentJoint)
   {
      this.transformToParentJoint = transformToParentJoint;
   }

   public void setYoAppearance(AppearanceDefinition yoAppearance)
   {
      this.yoAppearance = yoAppearance;
   }

   public void setCollisionGroup(int collisionGroup)
   {
      this.collisionGroup = collisionGroup;
   }

   public void setCollisionMask(int collisionMask)
   {
      this.collisionMask = collisionMask;
   }
}