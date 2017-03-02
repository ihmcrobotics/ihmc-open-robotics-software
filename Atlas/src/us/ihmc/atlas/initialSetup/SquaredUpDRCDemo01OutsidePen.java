package us.ihmc.atlas.initialSetup;

import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.euclid.tuple3D.Vector3D;

public class SquaredUpDRCDemo01OutsidePen extends OffsetAndYawRobotInitialSetup
{
   private static final Vector3D additionalOffset = new Vector3D(3.0, 12.0, 0.0);
   private static final double yaw = Math.PI/2.0;
   
   public SquaredUpDRCDemo01OutsidePen(double groundZ)
   {
      super(groundZ, additionalOffset, yaw);
   }
}

