package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class SensorDescription
{
   private String name;
   private final RigidBodyTransform transformToJoint = new RigidBodyTransform();

   public SensorDescription(String name, Vector3d offsetFromJoint)
   {
      this.name = name;
      transformToJoint.setTranslationAndIdentityRotation(offsetFromJoint);
   }

   public SensorDescription(String name, RigidBodyTransform transformToJoint)
   {
      this.name = name;
      this.transformToJoint.set(transformToJoint);
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setOffsetFromJoint(Vector3d offsetFromJoint)
   {
      transformToJoint.setTranslationAndIdentityRotation(offsetFromJoint);
   }

   public void getTransformToJoint(RigidBodyTransform transformToJointToPack)
   {
      transformToJointToPack.set(transformToJoint);
   }
   
   public void setTransformToJoint(RigidBodyTransform transformToJoint)
   {
      this.transformToJoint.set(transformToJoint);
   }

   public RigidBodyTransform getTransformToJoint()
   {
      return transformToJoint;
   }

   public Vector3d getOffsetFromJoint()
   {
      Vector3d ret = new Vector3d();

      transformToJoint.getTranslation(ret);

      return ret;
   }

}
