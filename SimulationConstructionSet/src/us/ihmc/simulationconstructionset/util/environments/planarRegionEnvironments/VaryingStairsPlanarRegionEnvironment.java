package us.ihmc.simulationconstructionset.util.environments.planarRegionEnvironments;

import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

public class VaryingStairsPlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   private double startX;
   private double startZ;
   private double[] stepTreads;
   private double[] stepRises;
   
   public VaryingStairsPlanarRegionEnvironment(double startX, double startZ, double[] stepTreads, double[] stepRises)
   {
      this.startX = startX;
      this.startZ = startZ;
      this.stepTreads = stepTreads;
      this.stepRises = stepRises;
      
      generateEnvironment();
   }

   @Override
   protected void buildGenerator(PlanarRegionsListGenerator generator)
   {
      generator.translate(startX, 0.0, startZ);
      
      for (int i = 0; i < stepTreads.length; i++)
      {
         generator.addCubeReferencedAtBottomNegativeXEdgeCenter(stepTreads[i], 1.0, -0.1);
         generator.translate(stepTreads[i] + 0.01, 0.0, stepRises[i]);
      }
   }
}
