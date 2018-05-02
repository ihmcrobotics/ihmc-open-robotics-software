package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.Axis;

public class ZigZagSlopeEnvironment extends PlanarRegionEnvironmentInterface
{
   public ZigZagSlopeEnvironment(double rampSlope, double rampLength, int numberOfRamps, double heightAtRampMiddle)
   {
      double rampLengthX = rampLength * Math.cos(rampSlope);
      double startX = -0.5 * rampLengthX * (numberOfRamps - 1);
      double slopeSign = -1.0;

      for (int i = 0; i < numberOfRamps; i++)
      {
         generator.identity();
         generator.translate(startX + i * rampLengthX, 0.0, heightAtRampMiddle);
         generator.rotate(rampSlope * slopeSign, Axis.Y);
         generator.addRectangle(rampLength, 1.0);
         slopeSign *= -1.0;
      }
   }
}
