package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Plane;

public class FloatingPlanarJointDescription extends JointDescription
{
   private Plane plane;

   public FloatingPlanarJointDescription(String name, Plane plane)
   {
      super(name, new Vector3d());
      this.setType(plane);
   }

   public Plane getPlane()
   {
      return plane;
   }

   public void setType(Plane plane)
   {
      this.plane = plane;
   }

}
