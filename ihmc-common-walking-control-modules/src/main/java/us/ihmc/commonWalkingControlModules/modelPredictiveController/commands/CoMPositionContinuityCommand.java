package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoefficientJacobianMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactStateMagnitudeToForceMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;

import java.util.ArrayList;
import java.util.List;

public class CoMPositionContinuityCommand extends CoMContinuityCommand
{
   public int getDerivativeOrder()
   {
      return 0;
   }
}
