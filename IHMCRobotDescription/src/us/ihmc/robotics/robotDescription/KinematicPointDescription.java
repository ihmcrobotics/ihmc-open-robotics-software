package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

public class KinematicPointDescription
{
   private String name;
   private Vector3d offsetFromJoint = new Vector3d();

   public KinematicPointDescription(String name, Vector3d offsetFromJoint)
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

   public void getOffsetFromJoint(Vector3d offsetFromJointToPack)
   {
      offsetFromJointToPack.set(offsetFromJoint);
   }

   public Vector3d getOffsetFromJoint()
   {
      return offsetFromJoint;
   }

   public void setOffsetFromJoint(Vector3d offsetFromJoint)
   {
      this.offsetFromJoint.set(offsetFromJoint);
   }

}
