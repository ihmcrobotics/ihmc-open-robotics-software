package us.ihmc.simulationconstructionset.util.ground;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public class DoubleStepGroundProfileTest extends GroundProfileTest
{
   @Override
   public GroundProfile3D getGroundProfile()
   {
      double yMin = -1.0;
      double yMax = 1.0;
      double initialElevationChangeX = 2.0;
      double finalElevationChangeX = 2.4;
      double initialElevationDifference = 0.1;
      double finalElevationDifference = 0.3;

      return new DoubleStepGroundProfile(yMin, yMax, initialElevationChangeX, finalElevationChangeX, initialElevationDifference, finalElevationDifference);
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testSurfaceNormalGridForSmoothTerrainUsingHeightMap()
   {
      super.testSurfaceNormalGridForSmoothTerrainUsingHeightMap();
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
