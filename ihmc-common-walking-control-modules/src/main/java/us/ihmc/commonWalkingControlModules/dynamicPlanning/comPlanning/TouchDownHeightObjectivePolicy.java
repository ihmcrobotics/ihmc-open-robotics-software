package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class TouchDownHeightObjectivePolicy implements CoMTrajectoryPlanningCostPolicy
{
   private final double weight;
   private final YoDouble omega;

   public TouchDownHeightObjectivePolicy(YoDouble omega, double weight)
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
         double c0 = CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega.getDoubleValue(), duration);
         double c1 = CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega.getDoubleValue(), duration);
         double c2 = CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(duration);
         double c3 = CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(duration);
         double c4 = CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(duration);
         double c5 = CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction();
         int startIdx = 6 * sequenceId;
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx + 0, weight * c0 * c0);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx + 1, weight * c0 * c1);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx + 2, weight * c0 * c2);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx + 3, weight * c0 * c3);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx + 4, weight * c0 * c4);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx, startIdx + 5, weight * c0 * c5);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx + 0, weight * c1 * c0);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx + 1, weight * c1 * c1);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx + 2, weight * c1 * c2);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx + 3, weight * c1 * c3);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx + 4, weight * c1 * c4);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 1, startIdx + 5, weight * c1 * c5);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 2, startIdx + 0, weight * c2 * c0);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 2, startIdx + 1, weight * c2 * c1);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 2, startIdx + 2, weight * c2 * c2);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 2, startIdx + 3, weight * c2 * c3);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 2, startIdx + 4, weight * c2 * c4);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 2, startIdx + 5, weight * c2 * c5);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 3, startIdx + 0, weight * c3 * c0);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 3, startIdx + 1, weight * c3 * c1);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 3, startIdx + 2, weight * c3 * c2);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 3, startIdx + 3, weight * c3 * c3);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 3, startIdx + 4, weight * c3 * c4);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 3, startIdx + 5, weight * c3 * c5);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 4, startIdx + 0, weight * c4 * c0);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 4, startIdx + 1, weight * c4 * c1);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 4, startIdx + 2, weight * c4 * c2);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 4, startIdx + 3, weight * c4 * c3);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 4, startIdx + 4, weight * c4 * c4);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 4, startIdx + 5, weight * c4 * c5);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx + 0, weight * c5 * c0);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx + 1, weight * c5 * c1);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx + 2, weight * c5 * c2);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx + 3, weight * c5 * c3);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx + 4, weight * c5 * c4);
         CoMTrajectoryPlannerTools.addEquals(hessianToPack, startIdx + 5, startIdx + 5, weight * c5 * c5);

         double copHeight = contactSequence.get(sequenceId + 1).getCopStartPosition().getZ();
         double objectiveCoMHeight = copHeight + comTrajectoryPlanner.getNominalCoMHeight();

         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx, 0, weight * objectiveCoMHeight * c0);
         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx + 1, 0, weight * objectiveCoMHeight * c1);
         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx + 2, 0, weight * objectiveCoMHeight * c2);
         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx + 3, 0, weight * objectiveCoMHeight * c3);
         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx + 4, 0, weight * objectiveCoMHeight * c4);
         CoMTrajectoryPlannerTools.addEquals(zGradientToPack, startIdx + 5, 0, weight * objectiveCoMHeight * c5);

      }
   }
}
