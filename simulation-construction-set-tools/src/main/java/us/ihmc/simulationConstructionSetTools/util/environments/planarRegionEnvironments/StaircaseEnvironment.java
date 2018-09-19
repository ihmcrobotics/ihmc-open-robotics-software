package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

public class StaircaseEnvironment extends PlanarRegionEnvironmentInterface
{
   public StaircaseEnvironment(int numberOfSteps, double stepHeight, double stepLength)
   {
      this(numberOfSteps, stepHeight, stepLength, false);
   }

   public StaircaseEnvironment(int numberOfSteps, double stepHeight, double stepLength, boolean includeDown)
   {
      double startingBlockLength = 1.2;
      generator.identity();
      generator.addRectangle(startingBlockLength, 1.75);
      generator.translate(0.5 * (startingBlockLength - stepLength), 0.0, 0.0);

      for (int i = 0; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, 1.75, (i + 1) * stepHeight);
      }

      generator.translate(0.5 * (startingBlockLength + stepLength), 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(startingBlockLength, 1.75, numberOfSteps * stepHeight);

      if (includeDown)
      {
         generator.translate(0.5 * (startingBlockLength - stepLength), 0.0, 0.0);

         for (int i = 0; i < numberOfSteps; i++)
         {
            generator.translate(stepLength, 0.0, 0.0);
            generator.addCubeReferencedAtBottomMiddle(stepLength, 1.75, (numberOfSteps - i) * stepHeight);
         }
      }

      generator.translate(0.5 * (startingBlockLength + stepLength), 0.0, 0.0);
      generator.addRectangle(startingBlockLength, 1.75);
   }
}
