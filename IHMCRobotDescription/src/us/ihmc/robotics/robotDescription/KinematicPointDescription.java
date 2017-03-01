package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.Vector3D;

public class KinematicPointDescription
{
   private String name;
   private Vector3D offsetFromJoint = new Vector3D();

   public KinematicPointDescription(String name, Vector3D offsetFromJoint)
   {
      this.setName(name);
      this.offsetFromJoint.set(offsetFromJoint);
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void getOffsetFromJoint(Vector3D offsetFromJointToPack)
   {
      offsetFromJointToPack.set(offsetFromJoint);
   }

   public Vector3D getOffsetFromJoint()
   {
      return offsetFromJoint;
   }

   public void setOffsetFromJoint(Vector3D offsetFromJoint)
   {
      this.offsetFromJoint.set(offsetFromJoint);
   }

}
