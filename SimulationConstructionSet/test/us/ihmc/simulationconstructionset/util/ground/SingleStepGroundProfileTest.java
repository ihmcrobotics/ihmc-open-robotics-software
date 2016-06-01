package us.ihmc.simulationconstructionset.util.ground;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;


public class SingleStepGroundProfileTest extends GroundProfileTest
{
   @Override
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout=300000)
   public void testSurfaceNormalGridForSmoothTerrainUsingHeightMap()
   {
      super.testSurfaceNormalGridForSmoothTerrainUsingHeightMap();
   }
   
   @Override
   public GroundProfile3D getGroundProfile()
   {
      return new SingleStepGroundProfile(-10.0, 10.0, -5.0, 5.0, 2.0, 0.2);
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
      return 0.01;
   }
}
