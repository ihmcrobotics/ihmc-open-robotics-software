package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Plane;

public class FloatingPlanarJointDescription extends JointDescription
{
   private Plane plane;

   public FloatingPlanarJointDescription(String name, Plane plane)
   {
      super(name, new Vector3D());
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
