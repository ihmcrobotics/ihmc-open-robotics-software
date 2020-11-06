package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixSparseCSC;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class TakeOffHeightObjectivePolicy implements CoMTrajectoryPlanningCostPolicy
{
   private final DoubleProvider omega;
   private final double weight;

   public TakeOffHeightObjectivePolicy(DoubleProvider omega, double weight)
   {
      this.omega = omega;
      this.weight = weight;
   }

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

         double copHeight = contactSequence.get(sequenceId - 1).getCopEndPosition().getZ();
         double objectiveCoMHeight = copHeight + comTrajectoryPlanner.getNominalCoMHeight();
         CoMTrajectoryPlannerTools.addValueObjective(weight,
                                                     sequenceId,
                                                     omega.getValue(),
                                                     0.0,
                                                     objectiveCoMHeight,
                                                     CoMTrajectoryPlannerTools.comPositionCoefficientProvider,
                                                     CoMTrajectoryPlannerTools.comPositionCoefficientSelectedProvider,
                                                     hessianToPack,
                                                     zGradientToPack);
      }
   }
}
