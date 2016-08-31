package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

public class FloatingPlanarJointDescription extends JointDescription
{
   private FloatingPlanarJointPlane plane;

   public FloatingPlanarJointDescription(String name, RobotDescription robotDescription, FloatingPlanarJointPlane plane)
   {
      super(name, new Vector3d());
      this.setType(plane);
   }

   public FloatingPlanarJointPlane getPlane()
   {
      return plane;
   }

   public void setType(FloatingPlanarJointPlane plane)
   {
      this.plane = plane;
   }

   public static enum FloatingPlanarJointPlane{XY, YZ, XZ}

}
