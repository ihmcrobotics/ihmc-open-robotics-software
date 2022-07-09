package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

public class VelocityRotationEdgeCalculatorTest extends RotationEdgeCalculatorTest
{
   @Override
   public RotationEdgeCalculator getEdgeCalculator()
   {
      double breakFrequency = 1.0;
      double stableAngleThreshold = 3.0;
      double stablePositionThreshold = 0.5;
      int stableWindowSize = 5;
      return new VelocityRotationEdgeCalculator(side,
                                                soleFrame,
                                                () -> breakFrequency,
                                                () -> stableAngleThreshold,
                                                () -> stablePositionThreshold,
                                                () -> stableWindowSize,
                                                dt,
                                                registry,
                                                null);
   }
}
