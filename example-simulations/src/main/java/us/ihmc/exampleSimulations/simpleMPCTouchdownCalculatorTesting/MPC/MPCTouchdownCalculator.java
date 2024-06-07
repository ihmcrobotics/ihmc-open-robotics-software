package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.BPWPLanarWalkingRobot;
import us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.BPWPlanarWalkingRobotEstimates;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MPCTouchdownCalculator
{
   private final YoRegistry registry;

   private final YoDouble mpcTouchdownPositionX;
   private final YoDouble mpcTouchdownPositionY;

   private final AutomaticFootstepMPCMatrixCalculator mpcFootstepCalculator;

   private final YoVector2D desiredVelocityProvider;

   public MPCTouchdownCalculator(BPWPLanarWalkingRobot controllerRobot, RobotSide swingSide,
                                 YoDouble desiredWalkingSpeed, YoDouble desiredSideWalkingSpeed,
                                 BPWPlanarWalkingRobotEstimates estimates,
                                 MPCParameters parameters, YoGraphicsListRegistry graphicsListRegistry, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(swingSide.getLowerCaseName() + getClass().getSimpleName());

      FrameVector3D angularMomentumForMPCStuff = new FrameVector3D();
      angularMomentumForMPCStuff.setIncludingFrame(estimates.getCentroidalAngularMomentum());
      angularMomentumForMPCStuff.changeFrame(estimates.getCenterOfMassControlZUPFrame());

      desiredVelocityProvider = new YoVector2D(desiredWalkingSpeed, desiredSideWalkingSpeed);

      mpcFootstepCalculator = new AutomaticFootstepMPCMatrixCalculator(swingSide.getOppositeSide(), angularMomentumForMPCStuff, estimates, desiredVelocityProvider, parameters, parentRegistry, graphicsListRegistry);

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
