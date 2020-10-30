package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class TouchDownHeightObjectivePolicy implements CoMTrajectoryPlanningCostPolicy
{
   private final double weight;
   private final DoubleProvider omega;

   public TouchDownHeightObjectivePolicy(DoubleProvider omega, double weight)
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
      for (int sequenceId = 0; sequenceId < contactSequence.size() - 1; sequenceId++)
      {
         ContactStateProvider contactState = contactSequence.get(sequenceId);
         boolean isEndTouchDown = contactSequence.get(sequenceId + 1).getContactState().isLoadBearing() && !contactState.getContactState().isLoadBearing();
         if (!isEndTouchDown)
            continue;

         double duration = contactState.getTimeInterval().getDuration();

         double copHeight = contactSequence.get(sequenceId + 1).getCopStartPosition().getZ();
         double objectiveCoMHeight = copHeight + comTrajectoryPlanner.getNominalCoMHeight();
         CoMTrajectoryPlannerTools.addValueObjective(weight,
                                                     sequenceId,
                                                     omega.getValue(),
                                                     duration,
                                                     objectiveCoMHeight,
                                                     CoMTrajectoryPlannerTools.comPositionCoefficientProvider,
                                                     CoMTrajectoryPlannerTools.comPositionCoefficientSelectedProvider,
                                                     hessianToPack,
                                                     zGradientToPack);
      }
   }
}
