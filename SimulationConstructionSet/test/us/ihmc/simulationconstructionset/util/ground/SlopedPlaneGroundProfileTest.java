package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public class SlopedPlaneGroundProfileTest extends GroundProfileTest
{
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSurfaceNormalGridForSmoothTerrainUsingHeightMap()
   {
      super.testSurfaceNormalGridForSmoothTerrainUsingHeightMap();
   }
   
   @Override
   public GroundProfile3D getGroundProfile()
   {
      Vector3d surfaceNormal = new Vector3d(0.03, 0.27, 1.0);
      surfaceNormal.normalize();
      Point3d intersectionPoint = new Point3d(0.22, 2.2, -0.4);
      double maxXY = 10.0;
      return new SlopedPlaneGroundProfile(surfaceNormal, intersectionPoint, maxXY);
   }

   @Override
   public double getMaxPercentageOfAllowableValleyPoints()
   {
      return 0.0;
   }

   @Override
   public double getMaxPercentageOfAllowablePeakPoints()
   {
      return 0.0;
   }

   @Override
   public double getMaxPercentageOfAllowableDropOffs()
   {
      return 0.0;
   }
}
