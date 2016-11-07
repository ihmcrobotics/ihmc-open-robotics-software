package us.ihmc.atlas.initialSetup;

import javax.vecmath.Vector3d;

import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;

public class SquaredUpDRCDemo01OutsidePen extends OffsetAndYawRobotInitialSetup
{
   private static final Vector3d additionalOffset = new Vector3d(3.0, 12.0, 0.0);
   private static final double yaw = Math.PI/2.0;
   
   public SquaredUpDRCDemo01OutsidePen(double groundZ)
   {
      super(groundZ, additionalOffset, yaw);
   }
}

