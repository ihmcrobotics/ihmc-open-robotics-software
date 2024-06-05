package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MPCTouchdownCalculator
{
   private final YoRegistry registry;

   private final YoDouble mpcTouchdownPositionX;
   private final YoDouble mpcTouchdownPositionY;

   public MPCTouchdownCalculator(BPWPLanarWalkingRobot controllerRobot, RobotSide swingSide,
                                 YoDouble desiredWalkingSpeed, YoDouble desiredSideWalkingSpeed,
                                 MPCParameters mpcParameters, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(swingSide.getLowerCaseName() + getClass().getSimpleName());

      mpcTouchdownPositionX = new YoDouble("mpcTouchdownPositionX", registry);
      mpcTouchdownPositionY = new YoDouble("mpcTouchdownPositionY", registry);

      parentRegistry.addChild(registry);
   }

   private void computeDesiredTouchdownPositionX()
   {

   }

   public void computeDesiredTouchdownPositionY()
   {

   }

   public double computeAndReturnDesiredTouchdownPositionX()
   {
      computeDesiredTouchdownPositionX();
      return mpcTouchdownPositionX.getDoubleValue();
   }

   public double computeAndReturnDesiredTouchdownPositionY()
   {
      computeDesiredTouchdownPositionY();
      return mpcTouchdownPositionY.getDoubleValue();
   }
}
