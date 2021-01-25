package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AngularMomentumECMPCalculator
{
   private final ECMPTrajectoryCalculator ecmpTrajectoryCalculator;
   private final ThreePotatoAngularMomentumCalculator angularMomentumCalculator;

   public AngularMomentumECMPCalculator(double mass, double potatoMass, double gravity,
                                        CenterOfMassJacobian centerOfMassJacobian,
                                        SideDependentList<MovingReferenceFrame> soleFrames,
                                        YoRegistry parentRegistry,
                                        YoGraphicsListRegistry graphicsListRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      ecmpTrajectoryCalculator = new ECMPTrajectoryCalculator(mass, gravity, registry);
      angularMomentumCalculator = new ThreePotatoAngularMomentumCalculator(gravity, potatoMass, centerOfMassJacobian, soleFrames, registry, graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

}
