package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixSparseCSC;

import java.util.List;

public interface CoMTrajectoryPlanningCostPolicy
{
   void assessPolicy(CoMTrajectoryPlannerInterface comTrajectoryPlanner,
                     List<? extends ContactStateProvider> contactSequence,
                     DMatrix hessianToPack,
                     DMatrix xGradientToPack,
                     DMatrix yGradientToPack,
                     DMatrix zGradientToPack);
}
