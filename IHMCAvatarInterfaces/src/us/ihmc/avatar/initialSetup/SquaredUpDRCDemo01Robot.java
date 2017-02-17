package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.tuple3D.Vector3D;

public class SquaredUpDRCDemo01Robot extends OffsetAndYawRobotInitialSetup
{
	   public SquaredUpDRCDemo01Robot(double offsetX, double offsetY)
	   {
	      super(new Vector3D(offsetX, offsetY, 0.0), Math.atan2(offsetY, offsetX));
	   }
	   public SquaredUpDRCDemo01Robot(double r, double theta, double psi)
	   {
		  super(new Vector3D(r*Math.cos(theta), r*Math.sin(theta), 0.0), theta+psi);
	   }
	   public SquaredUpDRCDemo01Robot(double z, double r, double theta, double psi)
	   {
		  super(z,new Vector3D(r*Math.cos(theta), r*Math.sin(theta), 0.0), theta+psi);
	   }
}
