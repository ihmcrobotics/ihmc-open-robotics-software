package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

public class FloatingJointDescription extends JointDescription
{
   private final String jointVariableName;

   public FloatingJointDescription(String name)
   {
      this(name, null);
   }

   public FloatingJointDescription(String name, String jointVariableName)
   {
      super(name, new Vector3d());
      this.jointVariableName = jointVariableName;
   }

   public String getJointVariableName()
   {
      return jointVariableName;
   }

}

