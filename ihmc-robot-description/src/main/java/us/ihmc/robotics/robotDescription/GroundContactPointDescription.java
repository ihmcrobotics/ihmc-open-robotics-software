package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.Vector3D;

public class GroundContactPointDescription extends ExternalForcePointDescription
{
   private final int groupIdentifier;

   public GroundContactPointDescription(String name, Vector3D offsetFromJoint)
   {
      super(name, offsetFromJoint);
      this.groupIdentifier = 0;
   }

   public GroundContactPointDescription(String name, Vector3D offsetFromJoint, int groupIdentifier)
   {
      super(name, offsetFromJoint);
      this.groupIdentifier = groupIdentifier;
   }

   public int getGroupIdentifier()
   {
      return groupIdentifier;
   }

}
