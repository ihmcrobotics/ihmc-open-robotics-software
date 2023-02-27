package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.robotics.robotSide.RobotSide;

public class CoPAndVelocityRotationEdgeCalculatorTest extends RotationEdgeCalculatorTest
{
   @Override
   protected RotationEdgeCalculator getEdgeCalculator()
   {
      double breakFrequency = 1.0;
      double copBreakFrequency = 30.0;
      double stableAngleThreshold = 3.0;
      double stablePositionThreshold = 0.5;
      int stableWindowSize = 5;
      return new CoPAndVelocityRotationEdgeCalculator(RobotSide.LEFT,
                                                      soleFrame,
                                                      () -> breakFrequency,
                                                      () -> copBreakFrequency,
                                                      () -> stableAngleThreshold,
                                                      () -> stablePositionThreshold,
                                                      () -> stableWindowSize,
                                                      dt,
                                                      registry,
                                                      null,
                                                      null);
   }


}
