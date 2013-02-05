package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ConstantCenterOfMassHeightTrajectoryGenerator implements CenterOfMassHeightTrajectoryGenerator
{
   private static final FrameVector2d flatSlope = new FrameVector2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
   private static final FrameVector2d flatSecondDerivative = new FrameVector2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
   
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

   
   public void solve(CenterOfMassHeightOutputData centerOfMassHeightOutputDataToPack, CenterOfMassHeightInputData centerOfMassHeightInputData)
   {
      centerOfMassHeightOutputDataToPack.setDesiredCenterOfMassHeight(desiredCenterOfMassHeight.getDoubleValue());
      centerOfMassHeightOutputDataToPack.setDesiredCenterOfMassHeightSlope(flatSlope);
      centerOfMassHeightOutputDataToPack.setDesiredCenterOfMassHeightSecondDerivative(flatSecondDerivative);
   }

   public double getDesiredCenterOfMassHeight()
   {
      return desiredCenterOfMassHeight.getDoubleValue();
   }

   public FrameVector2d getDesiredCenterOfMassHeightSlope()
   {
      return flatSlope;
   }

   public FrameVector2d getDesiredCenterOfMassHeightSecondDerivative()
   {
      return flatSlope;
   }
}
