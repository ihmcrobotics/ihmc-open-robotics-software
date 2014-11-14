package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile3D;

public class SlopedPlaneGroundProfileTest extends GroundProfileTest
{

   public GroundProfile3D getGroundProfile()
   {
      Vector3d surfaceNormal = new Vector3d(0.03, 0.27, 1.0);
      surfaceNormal.normalize();
      Point3d intersectionPoint = new Point3d(0.22, 2.2, -0.4);
      double maxXY = 10.0;
      return new SlopedPlaneGroundProfile(surfaceNormal, intersectionPoint, maxXY);
   }

   public double getMaxPercentageOfAllowableValleyPoints()
   {
      return 0.0;
   }

   public double getMaxPercentageOfAllowablePeakPoints()
   {
      return 0.0;
   }

   public double getMaxPercentageOfAllowableDropOffs()
   {
      return 0.0;
   }

   
}
