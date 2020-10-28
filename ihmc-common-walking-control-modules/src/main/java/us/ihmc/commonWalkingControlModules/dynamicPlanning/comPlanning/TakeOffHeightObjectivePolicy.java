package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixSparseCSC;

import java.util.List;

public class TakeOffHeightObjectivePolicy implements CoMTrajectoryPlanningCostPolicy
{
   double weight = 1.0;

   public void assessPolicy(CoMTrajectoryPlannerInterface comTrajectoryPlanner,
                            List<? extends ContactStateProvider> contactSequence,
                            DMatrix hessianToPack,
                            DMatrix xGradientToPack,
                            DMatrix yGradientToPack,
                            DMatrix zGradientToPack)
   {
      for (int sequenceId = 1; sequenceId < contactSequence.size(); sequenceId++)
      {
         ContactStateProvider contactState = contactSequence.get(sequenceId);
         boolean isStartTakeOff = contactSequence.get(sequenceId - 1).getContactState().isLoadBearing() && !contactState.getContactState().isLoadBearing();
         if (!isStartTakeOff)
            continue;

         // NOTE because this is being evaluated at t = 0.0, most of the hessian cost term is ZERO
         int startIdx = 6 * sequenceId;
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx, weight);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx + 1, weight);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx + 5, weight);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx, weight);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx + 1, weight);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx + 5, weight);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx, weight);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx + 1, weight);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx + 5, weight);

         double copHeight = contactSequence.get(sequenceId - 1).getCopEndPosition().getZ();
         double objectiveCoMHeight = copHeight + comTrajectoryPlanner.getNominalCoMHeight();

         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx, 0, weight * objectiveCoMHeight);
         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx + 1, 0, weight * objectiveCoMHeight);
         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx + 5, 0, weight * objectiveCoMHeight);

      }
   }
}
