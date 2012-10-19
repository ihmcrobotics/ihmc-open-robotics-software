package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.robotSide.RobotSide;

public class ConstantCenterOfMassHeightTrajectoryGenerator implements CenterOfMassHeightTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable desiredCenterOfMassHeight;

   public ConstantCenterOfMassHeightTrajectoryGenerator(YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);
      desiredCenterOfMassHeight = new DoubleYoVariable("desiredCenterOfMassHeight", registry); 
      desiredCenterOfMassHeight.set(1.15);
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

   public void setDesiredCenterOfMassHeight(double desiredCoMHeight)
   {
      desiredCenterOfMassHeight.set(desiredCoMHeight);
   }

}
