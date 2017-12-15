package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.RecyclingArrayList;

public interface LQRSolverInterface<E extends Enum>
{
   void setDesiredTrajectory(DiscreteOptimizationTrajectory desiredTrajectory, DenseMatrix64F initialState);
   void getOptimalTrajectory(DiscreteOptimizationTrajectory optimalTrajectoryToPack);

   DiscreteOptimizationTrajectory getOptimalTrajectory();
   DiscreteTrajectory getOptimalStateTrajectory();
   DiscreteTrajectory getOptimalControlTrajectory();
   RecyclingArrayList<DenseMatrix64F> getOptimalFeedbackGainTrajectory();
   RecyclingArrayList<DenseMatrix64F> getOptimalFeedForwardControlTrajectory();

   DenseMatrix64F getValueHessian();

   void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex); // backwards pass
   void computeOptimalTrajectories(E dynamicsState, int startIndex, int endIndex); // forward pass
}
