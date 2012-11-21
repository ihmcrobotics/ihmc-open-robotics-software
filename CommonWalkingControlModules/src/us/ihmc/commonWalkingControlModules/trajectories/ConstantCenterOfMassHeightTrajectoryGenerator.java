package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ConstantCenterOfMassHeightTrajectoryGenerator implements CenterOfMassHeightTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable desiredCenterOfMassHeight;

   public ConstantCenterOfMassHeightTrajectoryGenerator(double initialDesiredCoMHeight, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      desiredCenterOfMassHeight = new DoubleYoVariable("desiredCenterOfMassHeight", registry); 
      parentRegistry.addChild(registry);

      desiredCenterOfMassHeight.set(initialDesiredCoMHeight);
   }

   public void initialize(RobotSide supportLeg, RobotSide upcomingSupportLeg)
   {
      // empty
   }

   public void compute()
   {
      // empty
   }

   public double getDesiredCenterOfMassHeight()
   {
      return desiredCenterOfMassHeight.getDoubleValue();
   }

   public double getDesiredCenterOfMassHeightSlope()
   {
      return 0.0;
   }

   public double getDesiredCenterOfMassHeightSecondDerivative()
   {
      return 0.0;
   }
}
