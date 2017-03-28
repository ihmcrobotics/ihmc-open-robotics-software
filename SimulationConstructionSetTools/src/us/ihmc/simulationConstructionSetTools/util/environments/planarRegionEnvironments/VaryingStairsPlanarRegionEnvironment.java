package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

public class VaryingStairsPlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   public VaryingStairsPlanarRegionEnvironment(double startX, double startZ, double[] stepTreads, double[] stepRises)
   {
      generator.translate(startX, 0.0, startZ);
      
      for (int i = 0; i < stepTreads.length; i++)
      {
         generator.addCubeReferencedAtBottomNegativeXEdgeCenter(stepTreads[i], 1.0, -0.1);
         generator.translate(stepTreads[i] + 0.01, 0.0, stepRises[i]);
      }
   }
}
