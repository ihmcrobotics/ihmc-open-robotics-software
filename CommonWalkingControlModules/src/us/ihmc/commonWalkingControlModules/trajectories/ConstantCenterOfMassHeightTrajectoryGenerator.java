package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

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

   public void initialize(RobotSide supportLeg)
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

   public FrameVector2d getDesiredCenterOfMassHeightSlope()
   {
      return new FrameVector2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
   }

   public FrameVector2d getDesiredCenterOfMassHeightSecondDerivative()
   {
      return new FrameVector2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
   }
}
