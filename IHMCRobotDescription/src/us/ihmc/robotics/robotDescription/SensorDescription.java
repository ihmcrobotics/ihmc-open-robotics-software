package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class SensorDescription
{
   private String name;
   private final RigidBodyTransform transformToJoint = new RigidBodyTransform();

   public SensorDescription(String name, Vector3D offsetFromJoint)
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

   public void setOffsetFromJoint(Vector3D offsetFromJoint)
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

   public Vector3D getOffsetFromJoint()
   {
      Vector3D ret = new Vector3D();

      transformToJoint.getTranslation(ret);

      return ret;
   }

}
