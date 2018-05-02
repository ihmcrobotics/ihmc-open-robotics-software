package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.Axis;

public class SingleStepEnvironment extends PlanarRegionEnvironmentInterface
{
   public SingleStepEnvironment(double stepUpHeight, double stepLength)
   {
      // first ground plane
      generator.identity();
      generator.addRectangle(2.0, 2.0);

      // step
      generator.translate(1.0 + 0.5 * stepLength, 0.0, stepUpHeight);
      generator.addRectangle(stepLength, 2.0);

      generator.identity();
      generator.translate(1.0, 0.0, 0.5 * stepUpHeight);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(stepUpHeight, 2.0);

      generator.identity();
      generator.translate(1.0 + stepLength, 0.0, 0.5 * stepUpHeight);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(stepUpHeight, 2.0);

      // second ground plane
      generator.identity();
      generator.translate(2.0 + stepLength, 0.0, 0.0);
      generator.addRectangle(2.0, 2.0);
   }
}
